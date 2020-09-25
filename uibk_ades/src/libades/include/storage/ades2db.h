#pragma once

#include <vector>
#include "../types/ades2.h"
#include <boost/filesystem.hpp>
#include <iostream>
#include <type_traits>
#include <string>


namespace ades {

    class Ades2DB
    {
    private:
        std::string home_;

        unsigned int version_;

        std::vector<Ades2> ades_;

        bool populate();

        bool serialize();

    public:
        /*! Constructor for type ades::Ades2DB.
         *  \param name : home directory of <string> for this Ades2DB
         *  \param version : the version to use (if multiple version of the DB exist)
         */
        Ades2DB(std::string home, unsigned int version);

        ~Ades2DB();

        std::string getHome() const
        {
            return home_;
        }

        int getAdesNb()
        {
            return ades_.size();
        }

        /*! Verify by name if Ades2 exists in Ades2DB.
         *  \param name : an ADES name
         */
        bool isInDB(std::string name);
        /*! Adds a single Ades2 to this Ades2DB.
         *  \param ades : an ADES
         */
        void addAdes(Ades2 ades);


        /*! Adds a set of Ades2 to this Ades2DB.
         *  \param ades : a vector of <Ades2> to be added
         */
        void addAdes(std::vector<Ades2> ades);

        /*! Removes an Ades2 by name from this Ades2DB.
         *  \param name : a name of <string> of the Ades2 to remove
         */
        //void removeAdesByName(std::string name);
        bool removeAdesByName(std::string name);


        /*! Removes an Ades2 by ID from this Ades2DB.
         *  \param id : an id of <uint64_t> of the Ades2 to remove
         */
        void removeAdesByID(uint64_t id);


        /*! Update an Ades2 by provding its name.
         *  \param name : a name of <string> identifying the Ades2
         *  \return an iterator pointing to corresponding Ades2; if not found, it's a nullptr
         */
        std::vector<Ades2>::iterator updateAdesByName(std::string name);


        /*! Update an Ades2 by provding its ID.
         *  \param id : an ID of <uint64_t> identifying the Ades2
         *  \return an iterator pointing to corresponding Ades2; if not found, it's a nullptr
         */
        std::vector<Ades2>::iterator updateAdesByID(uint64_t id);


        /*! Return the current list of Ades2.
         *  \return a vector of <Ades2> containing all Ades2
         */
        std::vector<Ades2> listAdes() const
        {
            return ades_;
        }


        /*! Return an Ades2 by name. The returned Ades2 is not modifiable.
         *  \param name : a name of <string> identifying the Ades2
         *  \return the identified Ades2 or a NULL
         */
        const Ades2 getAdesByName(std::string name);


        /*! Return an Ades2 by ID. The returned Ades2 is not modifiable.
         *  \param id : an ID of <uint64_t> identifying the Ades2
         *  \return the identified Ades2 or a NULL
         */
        const Ades2 getAdesByID(uint64_t id);
    };
}
