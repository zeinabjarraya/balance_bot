#!/usr/bin/env python3
import time
import numpy as np
from balance_bot_env import BalanceBotEnv

if __name__ == "__main__":
    env = BalanceBotEnv(dt=0.02, max_effort=6.0, pitch_terminate_rad=0.6)
    try:
        for ep in range(3):
            obs, info = env.reset()
            print(f"Episode {ep} reset obs: {obs}")
            done = False
            t = 0
            while not done and t < 500:
                action = np.array([np.random.uniform(-0.2, 0.2)])  # small random torques
                obs, rew, term, trunc, info = env.step(action)
                if t % 50 == 0:
                    print(f"t={t:03d} obs={obs} rew={rew:.3f} term={term}")
                done = term or trunc
                t += 1
            print("Episode ended\n")
    finally:
        env.close()
