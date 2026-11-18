@echo off
setlocal

rem calls cmake to build the project
call "%~dp0CreateProject.bat" "return"
if errorlevel 1 (
    echo Project creation failed.
    goto DONE:
)

cd xragdoll_compiler.vs2022

cmake --build . --config Debug --target xragdoll_compiler
if errorlevel 1 (
    echo Debug compilation failed.
    goto DONE:
)

cmake --build . --config Release --target xragdoll_compiler
if errorlevel 1 (
    echo Release compilation failed.
    goto DONE:
)

rem if no one give us any parameters then we will pause it at the end, else we are assuming that another batch file called us
:DONE
if %1.==. pause
endlocal
