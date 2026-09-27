from .cli import main

if __name__ == "__main__":
    try:
        main()
    except (RuntimeError, ValueError, OSError, ImportError) as exc:
        raise SystemExit(f"Stopped: {exc}") from exc
