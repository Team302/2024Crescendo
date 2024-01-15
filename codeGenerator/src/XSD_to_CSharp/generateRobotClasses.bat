echo off
set ClassGenerator=.\XmlSchemaClassGenerator.2.0.732\XmlSchemaClassGenerator.Console.exe

set ROBOT_FILENAME=robot

set ROBOT_SCHEMA=%ROBOT_FILENAME%.xsd
set ROBOT_CSHARP=%ROBOT_FILENAME%.cs

set OUTPUT_RELATIVE_DIR=..\robotConfiguration

echo on

call %ClassGenerator% --output=%OUTPUT_RELATIVE_DIR% --pascal- -v %ROBOT_SCHEMA%

call replaceTextInFile.exe -r %OUTPUT_RELATIVE_DIR%\%ROBOT_CSHARP% "public\s+robot\(\)\s*{\s*" "$$COPY$$initialize();"
call replaceTextInFile.exe -r %OUTPUT_RELATIVE_DIR%\%ROBOT_CSHARP% "public\s+motor\(\)\s*{\s*" "$$COPY$$initialize();"