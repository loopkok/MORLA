"""Hold-to-jog MMK2 lift mapping shared by simulation and the SDK."""


class LiftButtonJog:
    requires_fresh_input = True
    release_on_input_loss = True

    def __init__(self, lower, upper, lead):
        self.lower, self.upper, self.lead = lower, upper, lead
        self.reset()

    def reset(self):
        self.armed = False
        self.direction = 0
        self.target = None

    def reanchor(self, value, measured, parameters):
        self.reset()
        self(value, measured, 0., parameters)
        return {'acknowledged': True, 'kind': 'mapping'}

    def __call__(self, value, measured, dt, parameters):
        if (not isinstance(value, (list, tuple)) or len(value) != 2
                or any(type(button) is not int or button not in (0, 1) for button in value)):
            raise ValueError('MMK2_LIFT_REQUIRES_QUEST_AB_BUTTONS')
        speed = parameters.get('speed', .01)
        if type(speed) not in (int, float) or not 0 < speed <= .02:
            raise ValueError('MMK2_LIFT_INVALID_JOG_SPEED')
        a, b = value
        position = min(self.upper, max(self.lower, measured[0]))
        if not a and not b:
            self.armed = True
        direction = b - a if self.armed else 0  # Positive slide_joint is down.
        if not direction:
            hold = self.direction != 0
            self.direction = 0
            self.target = None
            return [position] if hold else None
        if direction != self.direction:
            self.target = position
        self.direction = direction
        lower = max(self.lower, position - self.lead)
        upper = min(self.upper, position + self.lead)
        self.target = min(upper, max(lower, self.target + direction * speed * dt))
        return [self.target]
