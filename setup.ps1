Set-ExecutionPolicy RemoteSigned -Scope CurrentUser -Force

if (-not (Get-Command uv -ErrorAction SilentlyContinue)) {
    Write-Output "Installing uv..."
    irm https://astral.sh/uv/install.ps1 | iex
} else {
    Write-Output "uv already installed"
}

Write-Output "Creating virtual environment and installing dependencies..."
uv sync

Write-Output "Setup complete. In VS Code, select the interpreter .\\.venv\\Scripts\\python.exe"
