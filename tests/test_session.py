from src.app.session import RobotSession


def test_mock_session_reconnect_and_disconnect() -> None:
    session = RobotSession.from_config(mode="mock")

    session.connect()
    assert session.connected is True
    state = session.read_state()
    assert state.is_enabled is False

    session.enable()
    assert session.read_state().is_enabled is True

    session.disconnect()
    assert session.connected is False

    session.connect()
    assert session.connected is True
    session.close()
    assert session.connected is False


def test_mock_session_manual_home_uses_executor() -> None:
    session = RobotSession.from_config(mode="mock")
    session.connect()
    session.enable()

    session.send_joint_action([0.1, 0.1, 0.1, 0.1, 0.1, 0.1], 0.5)
    session.home()

    state = session.read_state()
    assert state.joint_position == [0.0] * 6
    assert state.gripper_position == 1.0
    session.close()


def test_mock_session_configures_master_slave_without_enable() -> None:
    session = RobotSession.from_config(mode="mock")
    session.connect()

    session.configure_master_slave(0xFC, 0x00, 0x00, 0x00)

    assert session.driver.master_slave_config == (0xFC, 0x00, 0x00, 0x00)
    assert session.read_state().is_enabled is False
    session.close()
