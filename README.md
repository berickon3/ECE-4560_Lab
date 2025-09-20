To setup:
Create repository folder
Open repository location (AVOID INSTALLING IN ONEDRIVE)
Right click, open command window in this directory
IN WINDOWS POWERSHELL -
    code .
Then, in VS code Powershell terminal
    Set-ExecutionPolicy RemoteSigned -scope CurrentUser      
    powershell -c "irm https://astral.sh/uv/install.ps1 | iex"

Restart Visual Studio Code
IN VSCODE POWERSHELL
    uv sync
UV will then install required packages and make a virtual environment

Within VS Code
    CTRL-SHIFT-P to open the command bar
    Python: Select Interpreter

Select the option that is .\venv\Scripts\python.exe which should be python 3.10.18

This should fully set up the environment and python to be interpreted correctly. Scripts and commands should be run appended to 'uv run'

Added MuJoCo to repository. Can be run using:
    uv run -m mujoco.viewer
    