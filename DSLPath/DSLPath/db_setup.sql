/*Setting up database for the DSLPath demonstration... */

/* Create guest user */
use mysql;
DELETE from user
	where User = 'DSLPath_guest';
INSERT INTO user
	(host, user, authentication_string, select_priv, insert_priv, update_priv, 
	delete_priv, create_priv, drop_priv, ssl_cipher, x509_issuer, x509_subject)
	values
	('%', 'DSLPath_guest', password(''), 'Y', 'Y', 'Y', 'Y', 'Y', 'Y', '', '', '');


/* Create database and declare schemas */
drop database if exists DSLPath_db;
create database DSLPath_db;
use DSLPath_db;
create table DSLPath_tbl (
	traj_id BIGINT NOT NULL AUTO_INCREMENT,
	trajname VARCHAR(60) NULL,
	destraj BLOB not NULL,
	curr_time VARCHAR(60) NULL,
	orig_range BLOB not NULL,
	PRIMARY KEY (traj_id));


/* Declare the path insertion procedure */
DROP PROCEDURE IF EXISTS sp_addPath;
DELIMITER $$
create definer='DSLPath_guest'@'%' procedure sp_addPath (
	IN p_trajname VARCHAR(60),
	IN p_destraj text,
	IN p_curr_time text,
	IN p_orig_range text
)
begin
	insert into DSLPath_tbl(
		trajname, destraj, curr_time, orig_range
	) values (
		p_trajname, p_destraj, p_curr_time, p_orig_range
	);
end$$
DELIMITER ;

/* Declare table for storing the resulting figures*/
drop table if exists DSLPath_res_nd;
create table DSLPath_res_nd (
	traj_id BIGINT NOT NULL AUTO_INCREMENT,
	trajname VARCHAR(600) NULL,
	est_traj BLOB not NULL,
	ref_traj BLOB not NULL,
	des_traj BLOB not NULL,
	avgerr REAL not NULL,
	phys_range BLOB not NULL,
	PRIMARY KEY(traj_id));

drop table if exists DSLPath_res_dnn;
create table DSLPath_res_dnn (
	traj_id BIGINT NOT NULL AUTO_INCREMENT,
	trajname VARCHAR(600) NULL,
	est_traj BLOB not NULL,
	ref_traj BLOB not NULL,
	des_traj BLOB not NULL,
	avgerr REAL not NULL,
	phys_range BLOB not NULL,
	PRIMARY KEY(traj_id));

/* Grant execution privileges to users */
grant EXECUTE on PROCEDURE sp_addPath to 'DSLPath_guest'@'%';
FLUSH PRIVILEGES;


