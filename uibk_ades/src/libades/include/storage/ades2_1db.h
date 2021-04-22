#pragma once

#include <vector>
#include "../types/ades2_1.h"
#include <boost/filesystem.hpp>
#include <iostream>
#include <type_traits>
#include <string>


namespace ades {

    class Ades2_1DB
    {
    private:
        std::string home_;

        unsigned int version_;

        std::vector<Ades2_1> ades_;

        bool populate();

        bool serialize();

    public:
        /*! Constructor for type ades::Ades2_1DB.
         *  \param name : home directory of <string> for this Ades2_1DB
         *  \param version : the version to use (if multiple version of the DB exist)
         */
        Ades2_1DB(std::string home, unsigned int version);

        ~Ades2_1DB();

        std::string getHome() const
        {
            return home_;
        }

        int getAdesNb()
        {
            return ades_.size();
        }

        /*! Verify by name if Ades2_1 exists in Ades2_1DB.
         *  \param name : an ADES name
         */
        bool isInDB(std::string name);
        /*! Adds a single Ades2_1 to this Ades2_1DB.
         *  \param ades : an ADES
         */
        void addAdes(Ades2_1 ades);


        /*! Adds a set of Ades2_1 to this Ades2_1DB.
         *  \param ades : a vector of <Ades2_1> to be added
         */
        void addAdes(std::vector<Ades2_1> ades);

        /*! Removes an Ades2_1 by name from this Ades2_1DB.
         *  \param name : a name of <string> of the Ades2_1 to remove
         */
        //void removeAdesByName(std::string name);
        bool removeAdesByName(std::string name);


        /*! Removes an Ades2_1 by ID from this Ades2_1DB.
         *  \param id : an id of <uint64_t> of the Ades2_1 to remove
         */
        void removeAdesByID(uint64_t id);


        /*! Update an Ades2_1 by provding its name.
         *  \param name : a name of <string> identifying the Ades2_1
         *  \return an iterator pointing to corresponding Ades2_1; if not found, it's a nullptr
         */
        std::vector<Ades2_1>::iterator updateAdesByName(std::string name);


        /*! Update an Ades2_1 by provding its ID.
         *  \param id : an ID of <uint64_t> identifying the Ades2_1
         *  \return an iterator pointing to corresponding Ades2_1; if not found, it's a nullptr
         */
        std::vector<Ades2_1>::iterator updateAdesByID(uint64_t id);


        /*! Return the current list of Ades2_1.
         *  \return a vector of <Ades2_1> containing all Ades2_1
         */
        std::vector<Ades2_1> listAdes() const
        {
            return ades_;
        }


        /*! Return an Ades2_1 by name. The returned Ades2_1 is not modifiable.
         *  \param name : a name of <string> identifying the Ades2_1
         *  \return the identified Ades2_1 or a NULL
         */
        const Ades2_1 getAdesByName(std::string name);


        /*! Return an Ades2_1 by ID. The returned Ades2_1 is not modifiable.
         *  \param id : an ID of <uint64_t> identifying the Ades2_1
         *  \return the identified Ades2_1 or a NULL
         */
        const Ades2_1 getAdesByID(uint64_t id);
    };
}
