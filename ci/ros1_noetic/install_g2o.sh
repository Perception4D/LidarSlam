#!/bin/sh

set -e

readonly g2o_repo="https://github.com/RainerKuemmerle/g2o"
readonly g2o_commit="20230806_git"

readonly g2o_root="$HOME/g2o"
readonly g2o_src="$g2o_root/src"
readonly g2o_build_root="$g2o_root/build"

git clone "$g2o_repo" "$g2o_src"
git -C "$g2o_src" checkout "$g2o_commit"

g2o_build () {
    local prefix="$1"
    shift

    cmake -GNinja \
        -S "$g2o_src" \
        -B "$g2o_build_root" \
        "-DCMAKE_INSTALL_PREFIX=$prefix" \
        -DG2O_USE_OPENGL:BOOL=OFF \
        -DG2O_USE_VENDORED_CERES:BOOL=OFF \
        -DG2O_BUILD_APPS:BOOL=OFF \
        -DCeres_DIR:FILEPATH=/usr/local/lib/cmake/Ceres \
        -DG2O_BUILD_EXAMPLES:BOOL=OFF \
        -DCMAKE_INSTALL_PREFIX:FILEPATH=/usr/local \
        "$@"
    cmake --build "$g2o_build_root" --target install
}

g2o_build /usr

rm -rf "$g2o_root"
