![](/Docs/dragon_302.png)
# 302CodeGenerator

The 302CodeGenerator is a .NET application which uses the robot definition xml files used by Team 302 in order to generate some of the C++ robot files.

## Why auto generate code?
All the information required to write the corresponding C++ classes is available in the xml files. So why not?
- Names are typed by a human only in one place (in the xml file)
  - Avoids typing mistakes
  - Gives the developers more time to think about more complicated features
- The code is and looks consistent
- After the base is setup, changes can be made quicker
- It is more fun (for some)


### Inputs to the code generator

- robot.xml -> the main robot definition file
- _mechanism_.xml -> the states definition of a mechanism. The name of the file should be the name of the mechanism. The file name is used in code generation to create some of the code fragments
- generatorConfig.xml
  - path of the main robot.xml file.
  - path where the generated code should be placed
  - templates for the generated files

### Outputs of the code generator

- Some of the robot C++ files. The list is still growing. When it stabilizes, will add the filenames here
