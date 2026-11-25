
import random

# The state machine itself: it holds the state and works out what to write each cycle.
class RandomWalkLogic:
    # Sets the starting state, the speeds and a seeded random number generator.
    def __init__(self, lv=1.0, av=1.0, v_randcoef=2.0, seed=0):
        self.lv, self.av = lv, av
        self.rng = random.Random(seed)
        # The initial transition i0 goes to Wait.
        self.state = "Wait"
        self.randcoef = v_randcoef
        self.sign = 1
        self.cycle = 0
        self.t_start = 0
        # Outputs produced by the previous cycle's exec step, written in the next cycle.
        self.pending = []

    # Enters Turn: picks a new random time and queues the turning command.
    def _enter_turn_state(self, out):
        self.state = "Turn"
        # The model's  randcoef = randomcoef ( )
        self.randcoef = self.rng.uniform(1.0, 5.0)
        # The model's  rwCmd.x = av * sign; rwCmd.y = 0; $ RWMove ! rwCmd
        out.append((self.av * self.sign, 0.0))

    # Enters Move_Forward: queues the forward command.
    def _enter_forward_state(self, out):
        self.state = "Move_Forward"
        # The model's  rwCmd.x = 0; rwCmd.y = lv; $ RWMove ! rwCmd
        out.append((0.0, self.lv))

    # Does one exec step (at most one transition) and returns the outputs it produces.
    def _run_one_exec_step(self, disable, enable):
        out = []

        # In Wait, Enable starts wandering (t2); otherwise stay in Wait (t1).
        if self.state == "Wait":
            if enable:
                # Wander's initial transition (t4) goes to Turn. The clock is not reset here.
                self._enter_turn_state(out)
            return out

        # Disable stops the robot and goes back to Wait (t3): rwCmd = (0, 0); $ RWMove ! rwCmd
        if disable:
            self.state = "Wait"
            out.append((0.0, 0.0))
            return out

        # Inside Wander, change state once the random time has passed: since(T) >= randcoef.
        expired = self.cycle - self.t_start >= self.randcoef
        if self.state == "Turn" and expired:
            # Turn to Move_Forward (t5): # T; randcoef = randomcoef ( ); sign = random_sign ( )
            self.t_start = self.cycle
            self.randcoef = self.rng.uniform(1.0, 5.0)
            self.sign = 1
            self._enter_forward_state(out)
        elif self.state == "Turn":
            # Turn to Turn (t8): the entry action runs again.
            self._enter_turn_state(out)
        elif expired:
            # Move_Forward to Turn (t6): # T; randcoef = randomcoef ( ), then the entry of Turn.
            self.t_start = self.cycle
            self.randcoef = self.rng.uniform(1.0, 5.0)
            self._enter_turn_state(out)
        else:
            # Move_Forward to Move_Forward (t7): the entry action runs again.
            self._enter_forward_state(out)
        return out

    # Runs one cycle and returns this cycle's RWMove writes as a list of (x, y).
    def run_one_cycle(self, disable, enable):
        self.cycle += 1
        written, self.pending = self.pending, self._run_one_exec_step(bool(disable), bool(enable))
        return written
