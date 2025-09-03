# ECE-4560 Lab

## Setup

After cloning the repository, open a PowerShell window in the project root and run:

```powershell
./setup.ps1
```

The script will:

- Set the PowerShell execution policy for the current user.
- Install [uv](https://docs.astral.sh/uv/) if it is not already installed.
- Create a virtual environment and install all project dependencies.

After the script finishes, open Visual Studio Code and run `Python: Select Interpreter` from the command palette. Choose the interpreter at `.\venv\Scripts\python.exe`.

All project commands should then be run prefixed with `uv run`.
