# FlyingGOAT

## Запуск

1. Зайти в терминале в эту папку (`FlyingGOAT`):

    ```bash
    cd FlyingGOAT
    ```

2. Если не установлены библиотеки, то установить их:

    ```bash
    pip install -r requirements.txt
    ```

3. Зайти через терминал на дроны (это нужно, чтобы добавить их в известные хосты):

    ```bash
    ssh pi@192.168.0.10
    ssh pi@192.168.0.20
    ```

4. Запустить бэкенд:

    ```bash
    python -m fastapi run src/main.py
    ```

5. Зайти на фронтенд:

    Открыть в браузере файл [front.html](front.html)
