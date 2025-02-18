FROM python:3.10

COPY pyproject.toml /app/pyproject.toml
WORKDIR /app
ENV PYTHONPATH=/app/src
RUN pip install -e .

COPY . /app

CMD ["python", "src/DRIVE_AGAIN/app.py"]