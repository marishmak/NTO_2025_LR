from pydantic_settings import BaseSettings


class Config(BaseSettings):
    drone_ip0: str = "192.168.2.76"
    # drone_ip0: str = "192.168.0.10"
    drone_port0: int = 22
    drone_mavlink_port0: int = 14550

    drone_ip1: str = "192.168.0.20"
    drone_port1: int = 22
    drone_mavlink_port1: int = 14550

    drone_username: str = "pi"
    drone_password: str = "s-6-zmKI"
    # drone_password: str = "raspberry"

    is_one_drone: bool = True
    # is_one_drone: bool = False

    selfcheck_timeout: int = 60


config = Config()
