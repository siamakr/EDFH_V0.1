#!/bin/bash

for i in {1..9}
do
	touch "sr_00$i.csv"
done

for n in {10..20}
do
	touch "sr_0$n.csv"
done
