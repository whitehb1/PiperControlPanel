from src.core.action import Action
from src.drivers.mock_driver import MockDriver


def test_mock_driver_updates_state_after_action() -> None:
    driver = MockDriver()
    driver.connect()
    driver.enable()
    driver.send_action(Action([0.1, 0.2, 0.3, 0.4, 0.5, 0.6], 0.25))

    state = driver.get_state()
    assert state.joint_position == [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]
    assert state.gripper_position == 0.25
    assert state.is_enabled is True


def test_mock_driver_records_master_slave_config_without_motion() -> None:
    driver = MockDriver()
    driver.connect()
    before = driver.get_state()

    driver.configure_master_slave(0xFA, 0x00, 0x00, 0x00)
    after = driver.get_state()

    assert driver.master_slave_config == (0xFA, 0x00, 0x00, 0x00)
    assert after.joint_position == before.joint_position
    assert after.gripper_position == before.gripper_position
    assert after.is_enabled == before.is_enabled
    assert driver.trajectory == []
