import xml.etree.ElementTree as ET
import logging

logging.basicConfig(level=logging.DEBUG, format='%(asctime)s - %(levelname)s - %(message)s')

def generate(file, umlToggle, csvToggle):
    # https://docs.python.org/3/library/xml.etree.elementtree.html
    logging.debug(f'Generator for {file}')
    tree = ET.parse(file)
    root = tree.getroot()
    printUML(umlToggle, '@startuml')
    printUML(umlToggle, '[*] --> Off')
    printCSV(csvToggle, 'Mechanism, State, TransitionsTo, TransitionNum')


    # for each mechanism Instance, find the name
    mechanismInstances = root.find('mechanismInstances')
    for mechanismInstance in mechanismInstances:
        mechanismInstance_name = mechanismInstance.find('name').text
        logging.debug(f'MechanismInstance: {mechanismInstance_name}')
        transitionNum = 0

        # Iterate through each mechanism within the mechanismInstance
        for mechanism in mechanismInstance.findall('mechanism'):
            mechanism_name = mechanism.find('name').text
            logging.debug(f'Mechanism: {mechanism_name}')

            # Iterate through each state within the mechanism
            for state in mechanism.find('states').findall('state'):
                state_name = state.find('name').text
                logging.debug(f'  State: {state_name}')

                # for each state, get what the motors are doing
                for motorTarget in state.find('motorTargets').findall('motorTarget'):
                    motorName = motorTarget.find('name').text
                    motorValue = motorTarget.find('target').find('value').text if motorTarget.find('target').find('value') is not None else '0'
                    motorUnits = motorTarget.find('target').find('__units__').text if motorTarget.find('target').find('__units__') is not None else ''
                    logging.debug(f'    Motor: {motorName} Value: {motorValue} Units: {motorUnits}')
                    printUML(umlToggle, f'{state_name} : {motorName} = {motorValue} {motorUnits}')

                # Iterate through each transition within the state
                for transitionState in state.find('transitionsTo').findall('stringParameterConstInMechInstance'):
                    transitionState_name = transitionState.find('value').text
                    logging.debug(f'    TransitionsToState: {transitionState_name}')
                    printUML(umlToggle, f'{state_name} --> {transitionState_name} : {mechanism_name} {transitionNum}')
                    printCSV(csvToggle, f'{mechanism_name}, {state_name}, {transitionState_name}, {transitionNum}')
                    transitionNum = transitionNum + 1
    printUML(umlToggle,'@enduml')


def printUML(umlOn, output):
    if umlOn:
        print(output)

def printCSV(csvOn, output):
    if csvOn:
        print(output)

if __name__ == '__main__':
    # Configure logging

    logging.debug(f'CodeStateDiagramGenerator')
    xmlConfig = '../codeGenerator/configuration/RobotConfig/robots/CompBot302.xml'
    generate(xmlConfig, False, True)
