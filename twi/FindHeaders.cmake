# Function to find headers that include a specific type
function(find_headers HEADER_LIST TYPE_NAME)
    set(FOUND_HEADERS "")
    foreach(HEADER IN LISTS HEADER_LIST)
        string(TOUPPER ${HEADER} HEADER_UPPER)
        string(REPLACE "." "_" HEADER_VAR ${HEADER_UPPER})
        string(REPLACE "/" "_" HEADER_VAR ${HEADER_VAR})

        check_include_file(${HEADER} HEADER_FOUND_${HEADER_VAR})

        if(HEADER_FOUND_${HEADER_VAR})
            list(APPEND FOUND_HEADERS ${HEADER})
            message(STATUS "${HEADER} includes ${TYPE_NAME}")
        else()
            message(STATUS "${HEADER} does not include ${TYPE_NAME}")
        endif()
    endforeach()

    # Export the list of found headers with the TYPE_NAME as a prefix
    set(${TYPE_NAME}_FOUND_HEADERS ${FOUND_HEADERS} PARENT_SCOPE)
endfunction()