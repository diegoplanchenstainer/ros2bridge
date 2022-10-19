"""
Settings for both ROS and WS.

TORNADO_SETTINGS: WS Settings.
WEBSOCKET_ADDRESS: WS Url.

get_tornado_settings: Get setting of the websocket.
get_ip: Get IP address for hosting ws.
"""

import socket
from typing import Dict

TORNADO_SETTINGS = {
    'websocket_ping_interval': 10,
    'websocket_ping_timeout': 30,
}

WEBSOCKET_ADDRESS = {
    'port': '9020',
    'address': 'localhost'
}

SHUTDOWN_TIMEOUT = 30


def get_tornado_settings() -> Dict[str, int]:
    """
    Get websocket settings.

    Returns:
        Dict[str, int]: Websocket settings.
    """
    return TORNADO_SETTINGS


def get_ip(ngrok=False) -> Dict[str, str]:
    """
    Get ip address of the machine.

    Returns:
        Dict[str, str]: Return ip address if found else 'localhost'.
    """
    if ngrok:
        WEBSOCKET_ADDRESS['address'] = '*'
        return WEBSOCKET_ADDRESS

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.connect(('8.8.8.8', 53))
    host: str = sock.getsockname()[0]
    sock.close()

    if host.startswith('192.168'):
        WEBSOCKET_ADDRESS['address'] = host

    return WEBSOCKET_ADDRESS