from fastapi import FastAPI, WebSocket, WebSocketDisconnect, Request
from fastapi.responses import HTMLResponse
from fastapi.staticfiles import StaticFiles
from fastapi.templating import Jinja2Templates
import json
import asyncio
import time
from app.services.fuzzy_control import ElevatorFuzzyController
from app.services.socket import ConnectionManager
from app.services.elevator_mqtt import ElevatorMQTT

import threading


app = FastAPI()

app.mount("/static", StaticFiles(directory="static"), name="static")
templates = Jinja2Templates(directory="templates")

mqtt_client = None
controller = ElevatorFuzzyController()
movement_data = []
current_status = {
    "current_floor": "ground",
    "current_position": 4.0,
    "target_floor": None,
    "is_moving": False,
    "direction": "stopped",
}

message_queue = asyncio.Queue()

manager = ConnectionManager()


def position_update_handler(data):
    global movement_data, current_status

    current_status.update(
        {
            "current_floor": data.get("current_floor", current_status["current_floor"]),
            "current_position": data.get(
                "current_position", current_status["current_position"]
            ),
            "target_floor": data.get("target_floor", current_status["target_floor"]),
            "is_moving": data.get("is_moving", current_status["is_moving"]),
            "direction": data.get("direction", current_status["direction"]),
        }
    )

    movement_data.append(
        {
            "timestamp": data.get("timestamp", time.time()),
            "position": data.get("current_position", 0),
            "target_position": data.get("target_position", 0),
            "motor_power": data.get("motor_power", 0),
            "error": data.get("error", 0),
        }
    )
    if len(movement_data) > 1000:
        movement_data = movement_data[-1000:]

    try:
        message_queue.put_nowait({"type": "position_update", "data": data})
    except asyncio.QueueFull:
        print("Message queue full, dropping position update")


def status_update_handler(data):
    global current_status

    current_status.update(data)

    try:
        message_queue.put_nowait({"type": "status_update", "data": data})
    except asyncio.QueueFull:
        print("Message queue full, dropping status update")


async def message_broadcaster():
    while True:
        try:
            message = await message_queue.get()

            await manager.broadcast(json.dumps(message))

            message_queue.task_done()

        except Exception as e:
            print(f"Error in message broadcaster: {e}")
            await asyncio.sleep(0.1)


def initialize_mqtt():
    global mqtt_client

    mqtt_client = ElevatorMQTT()

    mqtt_client.position_callback = position_update_handler
    mqtt_client.status_callback = status_update_handler

    if mqtt_client.connect():
        print("Conectado ao MQTT broker")
        return True
    else:
        print("Falha ao conectar ao cliente MQTT")
        return False


@app.on_event("startup")
async def startup_event():
    asyncio.create_task(message_broadcaster())

    def init_mqtt():
        initialize_mqtt()

    mqtt_thread = threading.Thread(target=init_mqtt, daemon=True)
    mqtt_thread.start()


@app.on_event("shutdown")
async def shutdown_event():
    if mqtt_client:
        mqtt_client.disconnect()


@app.get("/", response_class=HTMLResponse)
async def read_root(request: Request):
    floor_positions = controller.floor_positions
    available_floors = ["ground"] + [f"floor_{i}" for i in range(1, 9)]

    return templates.TemplateResponse(
        "index.html",
        {
            "request": request,
            "available_floors": available_floors,
            "floor_positions": floor_positions,
            "current_status": current_status,
        },
    )


@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    await manager.connect(websocket)

    try:
        await manager.send_personal_message(
            json.dumps({"type": "initial_status", "data": current_status}), websocket
        )

        await manager.send_personal_message(
            json.dumps(
                {
                    "type": "movement_data",
                    "data": movement_data[-100:] if movement_data else [],
                }
            ),
            websocket,
        )

        while True:
            data = await websocket.receive_text()
            try:
                message = json.loads(data)
                await handle_websocket_message(message, websocket)
            except json.JSONDecodeError:
                await manager.send_personal_message(
                    json.dumps({"type": "error", "message": "Invalid JSON format"}),
                    websocket,
                )

    except WebSocketDisconnect:
        manager.disconnect(websocket)


async def handle_websocket_message(message: dict, websocket: WebSocket):
    message_type = message.get("type")

    if message_type == "floor_request":
        floor = message.get("floor")
        if floor and mqtt_client:
            success = mqtt_client.move_to_floor(floor)
            await manager.send_personal_message(
                json.dumps(
                    {
                        "type": "floor_request_response",
                        "success": success,
                        "floor": floor,
                        "message": f"{'Movement started' if success else 'Movement failed'} to floor {floor}",
                    }
                ),
                websocket,
            )
        else:
            await manager.send_personal_message(
                json.dumps(
                    {
                        "type": "floor_request_response",
                        "success": False,
                        "message": "Invalid floor or MQTT client not available",
                    }
                ),
                websocket,
            )

    elif message_type == "get_status":
        await manager.send_personal_message(
            json.dumps({"type": "status_update", "data": current_status}), websocket
        )
