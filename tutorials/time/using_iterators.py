from opensourceleg.time import SoftRealtimeLoop


def main():
    # Create a 10Hz loop (0.1 second period)
    rt_loop = SoftRealtimeLoop(dt=0.1)

    print("Counting up every 0.1 seconds...")

    # Using the loop as an iterator
    for t in rt_loop:
        print(f"Time: {t:.3f} seconds")


if __name__ == "__main__":
    main()
