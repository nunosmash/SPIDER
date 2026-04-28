@echo off
echo === HEX → UF2 변환 시작 ===

if "%~1"=="" (
echo 파일을 드래그해서 실행하세요.
pause
exit
)

py uf2conv.py "%~1" -o "%~dpn1.uf2" -f 0xADA52840

echo === 변환 완료 ===
pause
