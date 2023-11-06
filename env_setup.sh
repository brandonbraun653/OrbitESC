poetry config virtualenvs.in-project true
poetry config virtualenvs.path "$(pwd)"/.venv
poetry config virtualenvs.create true
poetry env use python3.10
poetry install --no-interaction

echo "$(pwd)" > ./.venv/lib/python3.10/site-packages/orbitesc.pth