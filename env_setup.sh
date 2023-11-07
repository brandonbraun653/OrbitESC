poetry config virtualenvs.in-project true
poetry config virtualenvs.path "$(pwd)"/.venv
poetry config virtualenvs.create true
poetry env use python3.10
poetry install --no-interaction

# Delete orbitesc.pth if it exists
if [ -f ./.venv/lib/python3.10/site-packages/orbitesc.pth ]; then
  rm ./.venv/lib/python3.10/site-packages/orbitesc.pth
fi

touch ./.venv/lib/python3.10/site-packages/orbitesc.pth

# Add local paths to orbitesc.pth
echo "$(pwd)" >> ./.venv/lib/python3.10/site-packages/orbitesc.pth
echo "$(pwd)"/lib/Aurora/lib/nanopb/nanopb/generator >> ./.venv/lib/python3.10/site-packages/orbitesc.pth
echo "$(pwd)"/lib/Aurora/lib/nanopb/nanopb/generator/proto >> ./.venv/lib/python3.10/site-packages/orbitesc.pth