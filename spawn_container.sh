cname=OrbitESC_Embedded

if [ ! "$(docker ps -q -f name=$cname)" ]; then

    if [ "$(docker ps -aq -f status=exited -f name=$cname)" ]; then
        echo "Starting previously stopped container $cname"
        docker start -i $cname
        exit 0
    fi

    echo "Container $cname not found. Creating and starting it."
    docker run -it --user 1000:1000 --device=/dev/ttyUSB0 --device=/dev/ttyACM0 --device=/dev/ttyACM1 --name $cname --mount type=bind,source="$(pwd)",target=/usr/project arm_none_eabi
else
  echo "Container $cname already running"
fi
