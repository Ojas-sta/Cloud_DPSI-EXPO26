#!/usr/bin/env python3
"""
Unitree Go2 Posture & State Controller
Venue: DPSI EXPO-2026 | Platform: Unitree Go2 Edu Quadruped ("Cloud Argus")

Safely transitions Unitree Go2 between:
  • STAND: RecoveryStand -> StandUp -> BalanceStand (out of prone mode)
  • BALANCE: Active dynamic stabilization
  • PRONE / DAMP: Safe passive rest on floor
  • SIT / RISE_SIT: Presentation posture
  • HELLO: Wave greeting to audience

Usage:
  python3 Day1_Testes/control_posture.py --stand [--net-if en11]
  python3 Day1_Testes/control_posture.py --sit
  python3 Day1_Testes/control_posture.py --damp
  python3 Day1_Testes/control_posture.py --hello
"""

import sys
import time
import argparse

try:
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize
    from unitree_sdk2py.go2.sport.sport_client import SportClient
    HAS_SDK = True
except ImportError:
    HAS_SDK = False


def main():
    parser = argparse.ArgumentParser(description="Unitree Go2 Posture & Motion State Controller")
    parser.add_argument("--stand", action="store_true", help="Bring robot from prone/damping into BALANCE_STAND")
    parser.add_argument("--sit", action="store_true", help="Transition robot into sitting rest")
    parser.add_argument("--damp", action="store_true", help="Put robot into prone/damping mode on belly")
    parser.add_argument("--recovery", action="store_true", help="Execute RecoveryStand from uneven posture/fall")
    parser.add_argument("--hello", action="store_true", help="Perform friendly paw wave greeting")
    parser.add_argument("--net-if", type=str, default="en11", help="Network interface (default: en11 on macOS USB-ETH)")
    args = parser.parse_args()

    if not HAS_SDK:
        print("❌ Error: unitree_sdk2py is not installed in this environment.")
        sys.exit(1)

    print("=" * 70)
    print("🐾 UNITREE GO2 POSTURE CONTROLLER")
    print(f"🔌 Initializing CycloneDDS on interface '{args.net_if}'...")
    print("=" * 70)

    try:
        ChannelFactoryInitialize(0, args.net_if)
        sc = SportClient()
        sc.SetTimeout(5.0)
        sc.Init()
        print("✅ SportClient connected successfully.")
    except Exception as e:
        print(f"❌ Failed to connect to SportClient: {e}")
        sys.exit(1)

    if args.stand or (not args.sit and not args.damp and not args.recovery and not args.hello):
        print("\n🚀 Step 1: Sending RecoveryStand (clearing prone/joint lock)...")
        ret1 = sc.RecoveryStand()
        print(f"   Response: {ret1}")
        time.sleep(1.2)

        print("🚀 Step 2: Sending StandUp...")
        ret2 = sc.StandUp()
        print(f"   Response: {ret2}")
        time.sleep(1.0)

        print("🚀 Step 3: Engaging BalanceStand (active 4-leg dynamic equilibrium)...")
        ret3 = sc.BalanceStand()
        print(f"   Response: {ret3}")
        print("\n✅ Go2 is now STANDING UP in active BALANCE_STAND mode (nominal height: 0.354m)!")

    elif args.recovery:
        print("\n🛡️ Sending RecoveryStand...")
        ret = sc.RecoveryStand()
        print(f"   Response: {ret}")
        time.sleep(1.0)
        sc.BalanceStand()
        print("✅ Recovery complete. Posture: BALANCE_STAND.")

    elif args.sit:
        print("\n🧘 Sending Sit command...")
        ret = sc.Sit()
        print(f"   Response: {ret}")
        print("✅ Go2 is now SITTING.")

    elif args.damp:
        print("\n🛑 Putting Go2 into prone / damping mode (belly resting on floor)...")
        sc.StandDown()
        time.sleep(0.5)
        ret = sc.Damp()
        print(f"   Response: {ret}")
        print("✅ Go2 motors damped. Robot safely in PRONE mode.")

    elif args.hello:
        print("\n👋 Sending Hello wave greeting...")
        ret = sc.Hello()
        print(f"   Response: {ret}")


if __name__ == "__main__":
    main()
