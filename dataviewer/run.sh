#!/bin/bash

trap 'pkill -P $$' SIGINT SIGTERM

cd server
flask run &

cd ..
cd client
npm start &

wait