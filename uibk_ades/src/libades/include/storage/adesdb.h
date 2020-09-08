#pragma once

#include <vector>
#include "../types/ades.h"
#include <boost/filesystem.hpp>
#include <iostream>
#include <type_traits>
#include <string>


namespace ades {

    class AdesDB
    {
    private:
        std::string home_;

        unsigned int version_;

        std::vector<Ades> ades_;

        bool populate();

        bool serialize();

    public:
        /*! Constructor for type ades::AdesDB.
         *  \param name : home directory of <string> for this AdesDB
         *  \param version : the version to use (if multiple version of the DB exist)
         */
        AdesDB(std::string home, unsigned int version);

        ~AdesDB();

        std::string getHome() const
        {
            return home_;
        }

        int getAdesNb()
        {
            return ades_.size();
        }

        /*! Verify by name if Ades exists in AdesDB.
         *  \param name : an ADES name
         */
        bool isInDB(std::string name);
        /*! Adds a single Ades to this AdesDB.
         *  \param ades : an ADES
         */
        void addAdes(Ades ades);


        /*! Adds a set of Ades to this AdesDB.
         *  \param ades : a vector of <Ades> to be added
         */
        void addAdes(std::vector<Ades> ades);

        /*! Removes an Ades by name from this AdesDB.
         *  \param name : a name of <string> of the Ades to remove
         */
        //void removeAdesByName(std::string name);
        bool removeAdesByName(std::string name);


        /*! Removes an Ades by ID from this AdesDB.
         *  \param id : an id of <uint64_t> of the Ades to remove
         */
        void removeAdesByID(uint64_t id);


        /*! Update an Ades by provding its name.
         *  \param name : a name of <string> identifying the Ades
         *  \return an iterator pointing to corresponding Ades; if not found, it's a nullptr
         */
        std::vector<Ades>::iterator updateAdesByName(std::string name);


        /*! Update an Ades by provding its ID.
         *  \param id : an ID of <uint64_t> identifying the Ades
         *  \return an iterator pointing to corresponding Ades; if not found, it's a nullptr
         */
        std::vector<Ades>::iterator updateAdesByID(uint64_t id);


        /*! Return the current list of Ades.
         *  \return a vector of <Ades> containing all Ades
         */
        std::vector<Ades> listAdes() const
        {
            return ades_;
        }


        /*! Return an Ades by name. The returned Ades is not modifiable.
         *  \param name : a name of <string> identifying the Ades
         *  \return the identified Ades or a NULL
         */
        const Ades getAdesByName(std::string name);


        /*! Return an Ades by ID. The returned Ades is not modifiable.
         *  \param id : an ID of <uint64_t> identifying the Ades
         *  \return the identified Ades or a NULL
         */
        const Ades getAdesByID(uint64_t id);
    };
}
