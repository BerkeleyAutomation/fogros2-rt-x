#!/bin/bash

trap 'pkill -P $$' SIGINT SIGTERM

cd server
flask run &

cd ..
cd client
npm install 
npm start &

wait