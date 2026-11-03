"""Signal ownership helpers extracted unchanged from the local controller."""

def hold_target_lights(lights, saved):
    """只延长目标灯的自动计时，不调用可能影响全图的 freeze API。

    手动 set_state 仍负责相位切换。十亿秒远大于单次实验长度；退出时恢复原配时。
    saved 在每次修改前填入，保证部分初始化失败时仍能恢复已修改的灯。
    """
    for light in lights:
        saved.append((light, light.get_state(), light.get_green_time(),
                      light.get_yellow_time(), light.get_red_time()))
        light.set_green_time(1e9)
        light.set_yellow_time(1e9)
        light.set_red_time(1e9)

def restore_target_lights(saved):
    for light, state, green, yellow, red in saved:
        try:
            light.set_green_time(green)
            light.set_yellow_time(yellow)
            light.set_red_time(red)
            light.set_state(state)
        except RuntimeError as exc:
            print(f"Warning: failed to restore signal {light.id}: {exc}")
