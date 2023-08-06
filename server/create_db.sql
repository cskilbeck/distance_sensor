DROP DATABASE salt_sensor;

CREATE DATABASE salt_sensor;

USE salt_sensor;

CREATE TABLE devices (
        device_id INT UNSIGNED NOT NULL AUTO_INCREMENT,
        device_address VARCHAR(12) UNIQUE NOT NULL,
        device_name VARCHAR(64) UNIQUE,
        PRIMARY KEY(device_id)
);

CREATE TABLE readings (
        reading_id INT UNSIGNED NOT NULL AUTO_INCREMENT,
        device_id INT UNSIGNED NOT NULL,
        reading_vbat SMALLINT UNSIGNED NOT NULL,
        reading_distance SMALLINT UNSIGNED NOT NULL,
        reading_flags SMALLINT UNSIGNED NOT NULL,
        reading_timestamp TIMESTAMP NOT NULL,
        PRIMARY KEY(reading_id),
);

CREATE INDEX readings_index 
ON readings (device_id, reading_timestamp);
