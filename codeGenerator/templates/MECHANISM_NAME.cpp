$$_COPYRIGHT_$$
$$_GEN_NOTICE_$$

// FRC Includes
#include <networktables/NetworkTableInstance.h>


#include <$$_INCLUDE_PATH_$$/$$_MECHANISM_NAME_$$.h>

$$_MECHANISM_NAME_$$::$$_MECHANISM_NAME_$$()
{
}

void $$_MECHANISM_NAME_$$::Initialize()
{
    m_table = nt::NetworkTableInstance::GetDefault().GetTable(m_ntName);
    m_table.get()->PutBoolean("Enable Tuning for $$_MECHANISM_NAME_$$?", m_tuning);
}

void $$_MECHANISM_NAME_$$::Cyclic()
{
    CheckForTuningEnabled();
    if(m_tuning)
    {
        ReadTuningParamsFromNT();
    }
}

void $$_MECHANISM_NAME_$$::CheckForTuningEnabled()
{
    bool pastTuning = m_tuning;
    m_tuning = m_table.get()->GetBoolean("Enable Tuning for $$_MECHANISM_NAME_$$?", false);
    if(pastTuning != m_tuning && m_tuning == true)
    {
        PushTuningParamsToNT();
    }
}

void $$_MECHANISM_NAME_$$::ReadTuningParamsFromNT()
{
    $$_READ_TUNABLE_PARAMETERS_$$
}

void $$_MECHANISM_NAME_$$::PushTuningParamsToNT()
{
    $$_PUSH_TUNABLE_PARAMETERS_$$
}