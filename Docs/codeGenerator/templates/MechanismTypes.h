$$_COPYRIGHT_$$
$$_GEN_NOTICE_$$

#pragma once

//========================================================================================================
///	 @class			MechanismTypes
///  @brief      	This contains the enum for the mechanism types
//========================================================================================================
class MechanismTypes
{
public:
    //==================================================================================
    /// enum:           MECHANISM_TYPE
    /// description:    Indicates the type of mechanism
    //==================================================================================
    enum MECHANISM_TYPE
    {
        UNKNOWN_MECHANISM = -1,

        $$_MECHANISM_NAMES_ENUMS_$$
        
        MAX_MECHANISM_TYPES
    };

private:
    MechanismTypes() = delete;
    ~MechanismTypes() = delete;
};