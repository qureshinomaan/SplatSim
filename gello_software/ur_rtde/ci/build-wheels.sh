#!/bin/bash
set -e -x

export WORKDIR=$(pwd)
export PYHOME=/home
cd ${PYHOME}

/opt/python/cp36-cp36m/bin/pip3.6 install pip --upgrade
/opt/python/cp37-cp37m/bin/pip3.7 install pip --upgrade
/opt/python/cp38-cp38/bin/pip3.8 install pip --upgrade
/opt/python/cp39-cp39/bin/pip3.9 install pip --upgrade
/opt/python/cp310-cp310/bin/pip3.10 install pip --upgrade

/opt/python/cp37-cp37m/bin/pip install twine
/opt/python/cp37-cp37m/bin/pip install --prefer-binary cmake
ln -s /opt/python/cp37-cp37m/bin/cmake /usr/bin/cmake
cd ${WORKDIR}

# Collect the pythons
pys=(/opt/python/cp*/bin)

# Filter out Python 3.11, temporary until cibuildwheel CI update is committed
pys=(${pys[@]//*311*/})

# Compile wheels
for PYBIN in "${pys[@]}"; do
    "${PYBIN}/pip" wheel . -w wheelhouse/
    "${PYBIN}/python" setup.py sdist -d wheelhouse/
done

# Bundle external shared libraries into the wheels
for whl in wheelhouse/*$(uname -p).whl; do 
    auditwheel repair "$whl" -w wheelhouse/
    rm $whl
done

ls wheelhouse/

#  Upload
for WHEEL in wheelhouse/ur_rtde*; do
    # dev
    # /opt/python/cp37-cp37m/bin/twine upload \
    #     --skip-existing \
    #     --repository-url https://test.pypi.org/legacy/ \
    #     -u "${K8S_SECRET_TWINE_USERNAME}" -p "${K8S_SECRET_TWINE_PASSWORD}" \
    #     "${WHEEL}"
    # prod
    /opt/python/cp37-cp37m/bin/twine upload \
        --skip-existing \
        -u "${K8S_SECRET_TWINE_USERNAME}" -p "${K8S_SECRET_TWINE_PASSWORD}" \
        "${WHEEL}"
done
