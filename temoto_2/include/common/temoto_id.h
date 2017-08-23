#ifndef TEMOTO_ID_H
#define TEMOTO_ID_H

typedef int TemotoID;
#define unassigned_ID 0

class TemotoIDManager
{
public:

    /**
     * @brief checkID
     * @param ID_in
     * @return
     */
    TemotoID checkID (TemotoID ID_in)
    {
        if (ID_in == 0)
            return generateID();
        else
            return ID_in;
    }

private:

    /**
     * @brief current_ID_
     */
    TemotoID current_ID_ = 2150;

    /**
     * @brief generateID
     * @return
     */
    TemotoID generateID ()
    {
        incrementID();
        return current_ID_;
    }

    /**
     * @brief incrementID
     */
    void incrementID ()
    {
        // Increment the id and if it is "0" then increment again
        if (++current_ID_ == unassigned_ID)
            ++current_ID_;
    }
};

#endif
