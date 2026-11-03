"""
Project Cloud: Core Event Bus & Node Abstraction Layer
Provides a high-fidelity pure Python event bus and node abstraction
mirroring the standard ROS 2 rclpy.node.Node API.
Allows deterministic testing on macOS workstations and seamless deployment
on Ubuntu/Jetson environments.
"""

import threading
import time
import sys
from typing import Callable, Dict, List, Any, Optional, Type
from datetime import datetime

class CloudLogger:
    """Standardized logger matching rclpy Node logger interface."""
    def __init__(self, name: str):
        self.name = name

    def _log(self, level: str, msg: str, color_code: str = "\033[0m"):
        now = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        reset = "\033[0m"
        print(f"{color_code}[{level}] [{now}] [{self.name}]: {msg}{reset}", flush=True)

    def debug(self, msg: str):
        self._log("DEBUG", msg, "\033[90m")

    def info(self, msg: str):
        self._log("INFO", msg, "\033[36m")

    def warn(self, msg: str):
        self._log("WARN", msg, "\033[33m")

    def warning(self, msg: str):
        self.warn(msg)

    def error(self, msg: str):
        self._log("ERROR", msg, "\033[31m")


class CloudEventBus:
    """Thread-safe in-process publish/subscribe and request/response event bus."""
    _instance = None
    _lock = threading.Lock()

    def __new__(cls):
        with cls._lock:
            if cls._instance is None:
                cls._instance = super(CloudEventBus, cls).__new__(cls)
                cls._instance._subscribers: Dict[str, List[Callable[[Any], None]]] = {}
                cls._instance._services: Dict[str, Callable[[Any], Any]] = {}
                cls._instance._bus_lock = threading.RLock()
            return cls._instance

    @classmethod
    def reset(cls):
        """Reset all bus subscriptions (used in testing)."""
        with cls._lock:
            if cls._instance:
                with cls._instance._bus_lock:
                    cls._instance._subscribers.clear()
                    cls._instance._services.clear()

    def subscribe(self, topic: str, callback: Callable[[Any], None]):
        with self._bus_lock:
            if topic not in self._subscribers:
                self._subscribers[topic] = []
            if callback not in self._subscribers[topic]:
                self._subscribers[topic].append(callback)

    def unsubscribe(self, topic: str, callback: Callable[[Any], None]):
        with self._bus_lock:
            if topic in self._subscribers and callback in self._subscribers[topic]:
                self._subscribers[topic].remove(callback)

    def publish(self, topic: str, message: Any):
        with self._bus_lock:
            callbacks = list(self._subscribers.get(topic, []))
        for cb in callbacks:
            try:
                cb(message)
            except Exception as e:
                print(f"[CloudEventBus ERROR] Exception in subscriber for {topic}: {e}", file=sys.stderr)

    def register_service(self, service_name: str, callback: Callable[[Any], Any]):
        with self._bus_lock:
            self._services[service_name] = callback

    def unregister_service(self, service_name: str):
        with self._bus_lock:
            self._services.pop(service_name, None)

    def call_service(self, service_name: str, request: Any) -> Any:
        with self._bus_lock:
            srv = self._services.get(service_name)
        if srv is None:
            raise RuntimeError(f"Service '{service_name}' is not registered.")
        return srv(request)


class Publisher:
    def __init__(self, topic: str, msg_type: Type, bus: CloudEventBus):
        self.topic = topic
        self.msg_type = msg_type
        self.bus = bus

    def publish(self, msg: Any):
        self.bus.publish(self.topic, msg)


class Subscription:
    def __init__(self, topic: str, callback: Callable[[Any], None], bus: CloudEventBus):
        self.topic = topic
        self.callback = callback
        self.bus = bus
        self.bus.subscribe(topic, callback)

    def destroy(self):
        self.bus.unsubscribe(self.topic, self.callback)


class Service:
    def __init__(self, service_name: str, callback: Callable[[Any], Any], bus: CloudEventBus):
        self.service_name = service_name
        self.callback = callback
        self.bus = bus
        self.bus.register_service(service_name, callback)

    def destroy(self):
        self.bus.unregister_service(self.service_name)


class Client:
    def __init__(self, service_name: str, bus: CloudEventBus):
        self.service_name = service_name
        self.bus = bus

    def call(self, request: Any) -> Any:
        return self.bus.call_service(self.service_name, request)

    def wait_for_service(self, timeout_sec: float = 1.0) -> bool:
        start = time.time()
        while time.time() - start < timeout_sec:
            if self.service_name in self.bus._services:
                return True
            time.sleep(0.01)
        return self.service_name in self.bus._services


class Timer:
    def __init__(self, period_sec: float, callback: Callable[[], None]):
        self.period_sec = period_sec
        self.callback = callback
        self._running = True
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def _run(self):
        while self._running:
            start = time.time()
            try:
                self.callback()
            except Exception as e:
                print(f"[Timer ERROR] Exception in timer callback: {e}", file=sys.stderr)
            elapsed = time.time() - start
            sleep_time = max(0.0, self.period_sec - elapsed)
            time.sleep(sleep_time)

    def cancel(self):
        self._running = False


class CloudNode:
    """
    Standard Node base class.
    Drop-in compatible with rclpy.node.Node.
    """
    def __init__(self, node_name: str):
        self.node_name = node_name
        self.logger = CloudLogger(node_name)
        self.bus = CloudEventBus()
        self._publishers: List[Publisher] = []
        self._subscriptions: List[Subscription] = []
        self._services: List[Service] = []
        self._clients: List[Client] = []
        self._timers: List[Timer] = []

    def get_logger(self) -> CloudLogger:
        return self.logger

    def get_name(self) -> str:
        return self.node_name

    def create_publisher(self, msg_type: Type, topic: str, qos_profile: int = 10) -> Publisher:
        pub = Publisher(topic, msg_type, self.bus)
        self._publishers.append(pub)
        return pub

    def create_subscription(self, msg_type: Type, topic: str, callback: Callable[[Any], None], qos_profile: int = 10) -> Subscription:
        sub = Subscription(topic, callback, self.bus)
        self._subscriptions.append(sub)
        return sub

    def create_service(self, srv_type: Type, srv_name: str, callback: Callable[[Any], Any]) -> Service:
        srv = Service(srv_name, callback, self.bus)
        self._services.append(srv)
        return srv

    def create_client(self, srv_type: Type, srv_name: str) -> Client:
        client = Client(srv_name, self.bus)
        self._clients.append(client)
        return client

    def create_timer(self, timer_period_sec: float, callback: Callable[[], None]) -> Timer:
        t = Timer(timer_period_sec, callback)
        self._timers.append(t)
        return t

    def destroy_node(self):
        for t in self._timers:
            t.cancel()
        for sub in self._subscriptions:
            sub.destroy()
        for srv in self._services:
            srv.destroy()
        self.logger.info("Node destroyed cleanly.")
