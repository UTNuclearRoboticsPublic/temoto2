#ifndef TEMOTO_ID_H
#define TEMOTO_ID_H

namespace temoto_id
{

typedef int ID;
const ID UNASSIGNED_ID = 0;

class IDManager
{
public:

    /**
     * @brief checkID
     * @param ID_in
     * @return
     */
    ID checkID (ID ID_in)
    {
        if (ID_in == UNASSIGNED_ID)
        {
            return generateID();
        }
        else
        {
            return ID_in;
        }
    }

    /**
     * @brief generateID
     * @return
     */
    ID generateID ()
    {
        incrementID();
        return current_ID_;
    }

private:

    /**
     * @brief current_ID_
     */
    ID current_ID_ = 2150;


    /**
     * @brief incrementID
     */
    void incrementID ()
    {
        // Increment the id and if it is "0" then increment again
        if (++current_ID_ == UNASSIGNED_ID)
            ++current_ID_;
    }
};

}

// TODO: temporary mapping until code gets clean
namespace TemotoID = temoto_id;
#endif
