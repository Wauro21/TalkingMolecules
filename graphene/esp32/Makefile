AC=arduino-cli
CORE=esp32:esp32:esp32
PORT= /dev/ttyUSB0


default: build


FILE_TO_BUILD=esp32.ino


build: 
	@echo "[Building with arduino-cli]"
	@$(AC) compile --clean --fqbn $(CORE) $(FILE_TO_BUILD)


upload:
	@echo "[Uploading with arduino-cli]"
	@$(AC) upload -p $(PORT) --fqbn $(CORE) $(FILE_TO_BUILD)
