#!/bin/bash

find . -name *.h -o -name *.cxx -o -name *.hxx -o -name *.c | xargs clang-format-3.4 -i -style=file
