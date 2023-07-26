INSERT INTO `devices` (`device_name`)
SELECT * FROM (SELECT 'baz') AS tmp
WHERE NOT EXISTS (SELECT * FROM `devices` WHERE `device_name`='baz') LIMIT 1;
SELECT `device_id` FROM `devices` WHERE `device_name`='baz';