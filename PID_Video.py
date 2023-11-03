import matplotlib.pyplot as plt
import turtle
import time


TIMER = 0
TIME_STEP = 0.001
SETPOINT = 10
SIMULATION_TIME = 10
INITIAL_X = 0
INITIAL_Y = -100
MASS = 1  # kg
MAX_THRUST = 15  # Newtons
GRAVITATIONAL = -9.81
INITIAL_VELOCITY = 0
INITIAL_HEIGHT = 0

KP = 0.36
KI = 40.0
KD = 0.0008099999999999997

SCREEN_WIDTH = 800
SCREEN_HEIGHT = 600


class Rocket:
    def __init__(self):
        self.rocket = turtle.Turtle()
        self.initial_rocket_representation()
        self.ddy = 0
        self.dy = INITIAL_VELOCITY
        self.y = INITIAL_Y

    def initial_rocket_representation(self):
        self.rocket.shape("square")
        self.rocket.color("black")
        self.rocket.penup()
        self.rocket.goto(INITIAL_X, INITIAL_Y)
        self.rocket.speed(0)

    def set_ddy(self, thrust):
        self.ddy = GRAVITATIONAL + thrust / MASS

    def set_dy(self):
        self.dy += self.ddy * TIME_STEP

    def set_y(self):
        self.y += self.dy * TIME_STEP


class PID:
    def __init__(self, kp, ki, kd, anti_windup):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.anti_windup = anti_windup

        self.error = 0
        self.integral_error = 0
        self.error_last = 0
        self.derivative_error = 0

    def __call__(self, error):
        self.error = error
        self.derivative_error = (self.error - self.error_last) / TIME_STEP
        self.error_last = self.error
        output = (
            self.kp * self.error
            + self.ki * self.integral_error
            + self.kd * self.derivative_error
        )
        if (
            abs(output) >= MAX_THRUST
            and self.error >= 0
            and self.integral_error >= 0
            or self.error < 0
            and self.integral_error < 0
        ):
            if self.anti_windup:
                # no integration
                self.integral_error = self.integral_error
            else:
                # if no antiWindup rectangular integration
                self.integral_error += self.error * TIME_STEP
        else:
            # rectangular integration
            self.integral_error += self.error * TIME_STEP
        output = min(max(output, 0), MAX_THRUST)
        return output

    def get_kpe(self):
        return self.kp * self.error

    def get_kde(self):
        return self.kd * self.derivative_error

    def get_kie(self):
        return self.ki * self.integral_error


def setup_turtle():
    screen = turtle.Screen()
    screen.setup(width=SCREEN_WIDTH, height=SCREEN_HEIGHT)
    marker = turtle.Turtle()
    marker.penup()
    marker.left(180)
    marker.goto(15, SETPOINT)
    marker.color("red")


def check_for_abort(start_time, rocket):
    if time.monotonic() - start_time > SIMULATION_TIME:
        print("SIMULATION TIMEOUT")
        return True
    elif not -700 < rocket.y < 700:
        print("OUT OF BOUNDS")
        return True
    else:
        return False


def draw_graph(x, y1, y2, y3, y4, y5):
    fig, (ax1, ax2, ax3, ax4, ax5) = plt.subplots(5, sharex=True)
    # fig.suptitle('antiwindup')
    ax1.set(ylabel="rocket \nHeight")
    ax1.plot(x, y1)
    ax2.set(ylabel="KP_error")
    ax2.plot(x, y2, "tab:red")
    ax3.set(ylabel="KD_error")
    ax3.plot(x, y3, "tab:orange")
    ax4.set(ylabel="KI_error")
    ax4.plot(x, y4, "tab:pink")
    ax5.set(ylabel="rocket \nThrust")
    ax5.plot(x, y5, "tab:brown")
    plt.show()


def do_simulation(pid, rocket):
    poses = []
    times = []
    kp_values = []
    kd_values = []
    ki_values = []
    thrust_values = []
    start_time = time.monotonic()
    while True:
        position_error = SETPOINT - rocket.y
        thrust = pid(position_error)
        print(thrust)
        rocket.set_ddy(thrust)
        rocket.set_dy()
        rocket.set_y()
        time.sleep(TIME_STEP)
        if check_for_abort(start_time, rocket):
            break

        poses.append(rocket.y)
        times.append(time.monotonic() - start_time)
        kp_values.append(pid.kp)
        kd_values.append(pid.kd)
        ki_values.append(pid.ki)
        thrust_values.append(thrust)
    return times, poses, kp_values, kd_values, ki_values, thrust_values


def main():
    rocket = Rocket()
    pid = PID(KP, KI, KD, True)
    setup_turtle()
    times, poses, kp_values, kd_values, ki_values, thrust_values = do_simulation(
        pid, rocket
    )
    draw_graph(times, poses, kp_values, kd_values, ki_values, thrust_values)


if __name__ == "__main__":
    main()
