DROP DATABASE salt_sensor;

CREATE DATABASE salt_sensor;

USE salt_sensor;

CREATE TABLE devices (
        device_id INT NOT NULL AUTO_INCREMENT,
        device_name VARCHAR(128) UNIQUE,
        PRIMARY KEY(device_id)
);

CREATE TABLE readings (
        reading_id INT NOT NULL AUTO_INCREMENT,
        device_id INT NOT NULL,
        reading_vbat INT,
        reading_distance INT,
        reading_timestamp TIMESTAMP,
        PRIMARY KEY(reading_id)
);

CREATE INDEX readings_index 
ON readings (device_id, reading_timestamp);
