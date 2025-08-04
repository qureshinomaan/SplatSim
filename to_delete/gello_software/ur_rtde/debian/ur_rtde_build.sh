#!/bin/bash

getFolder () {
    HasRtde=$(expr $(pwd|grep "ur_rtde$" -c) + $(pwd|grep "Ur_RTDE$" -c))
    
    if [ $HasRtde -gt 0 ]
    then
        HasRtde=0
        while [ $(pwd|grep "ur_rtde$" -c) -gt 0 ] || [ $(pwd|grep "Ur_RTDE$" -c) -gt 0 ]
        do
            if [ $(pwd|grep "ur_rtde/ci$" -c) -gt 0 ] || [ $(pwd|grep "Ur_RTDE/ci$" -c) -gt 0 ]
            then
                cd ..
                continue
                
            fi
            if [ $(pwd|grep "ur_rtde$" -c) -gt 0 ] || [ $(pwd|grep "Ur_RTDE$" -c) -gt 0 ]
            then
                echo $(pwd)
                HasRtde=1
                break
            fi
            cd ..
        done
    fi
    
    if [ $HasRtde -eq 1 ]
    then
        return 0
    else
        if [[ -d $ur_rtde_DIR ]] ; then
            echo $ur_rtde_DIR
            return 0
        fi
        exit 1
    fi
}

getVersion () {
    version=""
    gv_regex="project\(\s*ur_rtde\s*VERSION\s*[0-9]*\.[0-9]*\.[0-9]*\s.*\)"
    gv_regex_num="[0-9]*\.[0-9]*\.[0-9]"
    while IFS= read -r line
    do
        if [[ "$line" =~ $gv_regex ]] ; then
            for arg in $line ; do
                if [[ $arg =~ $gv_regex_num ]] ; then
                    version=$arg
                    echo $version
                    return 0
                fi
            done
            break
        fi
    done < "$ur_rtde_DIR/CMakeLists.txt"

    exit 1
}

getDebRev () {
    gdr_regex="ur-rtde\s\([0-9]*\.[0-9]*\.[0-9]*-[0-9a-z]*\)"
    gdr_regex_num="\([0-9]*\.[0-9]*\.[0-9]*\-[0-9a-z]*\)"
    while IFS= read -r line
    do
        
        if [[ $line =~ $gdr_regex ]] ; then
            for arg in $line ; do 
                if [[ $arg =~ $gdr_regex_num ]] ; then
                     
                    newArg=${arg:1}
                    newArg=${newArg%?}
                    ver=${newArg%-*}
                    rev=${newArg#*-}

                    
                    if [[ $ur_rtde_VERSION == $ver ]] ; then 
                        if [[ $1 == "-n" ]] ; then
                            rev=$(expr $rev + 1)
                        fi
                    else 
                        rev=1
                        ver=$ur_rtde_VERSION
                    fi

                    echo "$ver-$rev"

                    return 0
                fi
            done
            break
        fi
        
    done < "$ur_rtde_DIR/debian/changelog"
    exit 1
}


gpgkey=$UR_RTDE_DEB_KEY
regex_key="^key"

ur_rtde_DIR=$(getFolder)
ur_rtde_VERSION=$(getVersion)

extra=""
if [[ -z $@ ]] ; then 
    extra=build
fi

for input in $@ $extra ; do
    cd $ur_rtde_DIR

    if [[ $input == "deb" ]] ; then
        
        if [[ ! -e "../ur-rtde_$ur_rtde_VERSION.orig.tar.gz" ]] ; then
            git clean -fdX
            tar --exclude="./debian" --exclude="./.git" -zcvf "../ur-rtde_$ur_rtde_VERSION.orig.tar.gz" . > /dev/null
        fi
        debuild --no-sign || exit 1

        exit 0

    elif [[ $input == "deb-upload" ]] ; then
        if [[ -z $gpgkey ]] ; then 
            echo "A key for signing the package needs to be provided"
            echo "  key=B4227B543B90FFC4C26096A646A520C64CB3D1A9"
            echo "  or export UR_RTDE_DEB_KEY with the key fingerprint"
            exit 1
        fi
        if [[ ! -e "../ur-rtde_$ur_rtde_VERSION.orig.tar.gz" ]] ; then
            git clean -fdX
            tar --exclude="./debian" --exclude="./.git" -zcvf "../ur-rtde_$ur_rtde_VERSION.orig.tar.gz" . > /dev/null
        fi
        gbp dch -R -N "$(getDebRev)" --debian-branch=master --upstream-branch=master
        debuild --no-sign -S || exit 1
        debsign -k $gpgkey ../ur-rtde_$(getDebRev)_source.changes || exit 1
        dput ur-rtde ../ur-rtde_$(getDebRev)_source.changes || exit 1

        exit 0
    elif [[ "$input"  == "-c" ]] ; then
        newVersion=$(getDebRev -n)
        gbp dch -R -N "$newVersion" --debian-branch=master --upstream-branch=master || exit 1

    elif [[ "$input" =~ $regex_key ]] ; then
        gpgkey=${input#*=}

    elif [[ -z $input ]] || [[ $input == "build" ]] ; then
        mkdir Build
        cd Build
        cmake -DPYTHON_BINDINGS=OFF .. || exit 1
        make -j$(nproc --all) || exit 1
        exit 0 
    elif [[ $input == "clean" ]] ; then 
        git clean -fdX
        rm ../ur-rtde*.build 2> /dev/null
        rm ../ur-rtde*.debian.tar.xz 2> /dev/null
        rm ../ur-rtde*.dsc 2> /dev/null
        rm ../librtde*.deb 2> /dev/null
        rm ../librtde*.ddeb 2> /dev/null
        rm ../ur-rtde*.buildinfo 2> /dev/null
        rm ../ur-rtde*.changes 2> /dev/null
    else
        echo "argument \"$input\" not understood"
        exit 1 
    fi
    
done