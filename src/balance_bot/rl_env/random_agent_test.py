#!/usr/bin/env python3
import time
import numpy as np

from balance_bot_env import BalanceBotEnv

def run_random_agent(episodes=3, max_steps=300, print_every=10):
    env = BalanceBotEnv(
        model_name="balance_bot",  # change if your model_name differs
        max_effort=10.0,           # match your controller limit
        dt=0.02,                   # 50 Hz control
        max_episode_steps=max_steps,
        pitch_terminate_rad=0.4    # ~23°, fine for testing
    )

    for ep in range(episodes):
        obs, info = env.reset()
        pitch_deg = np.degrees(obs[0])
        print(f"\nEpisode {ep} reset obs: pitch={pitch_deg:.2f}°, "
              f"rate={obs[1]:.3f}, lv={obs[2]:.3f}, rv={obs[3]:.3f}")

        ep_ret, ep_len = 0.0, 0
        terminated = False
        truncated = False

        # Optional tiny settle to let sensors tick once post-reset
        time.sleep(0.05)

        while not (terminated or truncated):
            # Random continuous action in [-1, 1]
            action = env.action_space.sample()      # shape (1,)
            obs, reward, terminated, truncated, _ = env.step(action)
            ep_ret += reward
            ep_len += 1

            # Print a compact line every few steps and on termination
            if (ep_len % print_every == 0) or terminated or truncated:
                print(f"t={ep_len:03d} "
                      f"pitch={np.degrees(obs[0]):6.2f}° "
                      f"rate={obs[1]: .2f} "
                      f"act={float(action[0]): .2f} "
                      f"rew={reward: .3f} "
                      f"term={terminated} trunc={truncated}")

        print(f"Episode {ep} summary: len={ep_len}, return={ep_ret:.3f}")

if __name__ == "__main__":
    run_random_agent(episodes=3, max_steps=300, print_every=10)
