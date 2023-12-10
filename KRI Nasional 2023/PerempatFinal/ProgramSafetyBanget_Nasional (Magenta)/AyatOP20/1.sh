#!/bin/bash
make clean ;
make reinstall ;
make install ;
cd project ;
cd demo ;
make clean ;
make ;
