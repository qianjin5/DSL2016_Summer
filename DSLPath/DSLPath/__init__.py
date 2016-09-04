from flask import Flask, render_template, json, request
from flaskext.mysql import MySQL
import json


app = Flask(__name__)
mysql = MySQL()
app.config['MYSQL_DATABASE_USER'] = 'DSLPath_guest'
app.config['MYSQL_DATABASE_AUTHENTICATION_STRING'] = ''
app.config['MYSQL_DATABASE_DB'] = 'DSLPath_db'
app.config['MYSQL_DATABASE_HOST'] = 'localhost'
mysql.init_app(app)


def add_traj_to_db(traj_name, des_traj, curr_time, trial_time):
	conn = mysql.connect()
	cursor = conn.cursor()
	cursor.callproc('sp_addPath', (traj_name, des_traj, curr_time, trial_time))
	data = cursor.fetchall()
	if len(data) == 0:
		conn.commit()
		cursor.close()
		conn.close()
		return json.dumps({'message': 'Desired path sumbitted successfully!'})
	else:
		cursor.close()
		conn.close()
		return json.dumps({'error': str(data[0])})

def queryResNum():
	global numResNoDNN, numResDNN, newResultSubmitted
	t1 = get_numResNoDNN()
	t2 = get_numResDNN()
	if t1 > numResNoDNN or t2 > numResDNN:
		newResultSubmitted = True

def get_numResNoDNN():
	conn = mysql.connect()
	cursor = conn.cursor()
	query = ('select count(traj_id) from DSLPath_res_nd where traj_id>0;')
	cursor.execute(query)
	for item in cursor:
		res = int(item[0])
	cursor.close()
	conn.close()
	return res

def get_numResDNN():
	conn = mysql.connect()
	cursor = conn.cursor()
	query = ('select count(traj_id) from DSLPath_res_dnn where traj_id>0;')
	cursor.execute(query)
	for item in cursor:
		res = int(item[0])
	cursor.close()
	conn.close()
	return res

def my_str(s):
	return "'" + s + "'"


@app.route("/", methods = ['GET', 'POST'])
def hello():
	return render_template('drawpad.html')

@app.route("/gallery", methods = ['GET', 'POST'])
def gallery():
	return render_template('gallery.html')

@app.route("/viewDetail", methods = ['GET'])
def viewDetail():
	return render_template('viewDetail.html')

@app.route("/des_path_submission", methods = ['POST'])
def submit():

	if request.method == 'POST':

		traj_name = request.args.get('traj_name')
		des_traj = request.args.get('des_traj')
		curr_time = request.args.get('curr_time')
		orig_range = request.args.get('orig_range')
		add_traj_to_db(traj_name, des_traj, curr_time, orig_range)
		res = "<p>I got your trajectory named " + str(traj_name) + "</p>"
		return res # this sentence goes to xhttp.responseText in action.js


@app.route("/query_for_result", methods = ['GET'])
def queryResult():
	numResNoDNN = get_numResNoDNN()
	numResDNN = get_numResDNN()
	res = {'numResNoDNN': numResNoDNN, 'numResDNN': numResDNN}
	return json.dumps(res)


@app.route("/get_trial_result", methods = ['GET'])
def getNewestTrialResult():
	hasdnn = request.args.get('hasdnn')
	conn = mysql.connect()
	cursor = conn.cursor()
	data = {}
	if hasdnn == 'true':
		query = ('''select traj_id, trajname, est_traj, ref_traj, des_traj, avgerr, phys_range from DSLPath_res_dnn
			where traj_id in (
			select MAX(traj_id) from DSLPath_res_dnn); ''')
		cursor.execute(query)

		for traj_id, trajname, est_traj, ref_traj, des_traj, avgerr, phys_range in cursor:
			data['traj_id'] = traj_id
			data['trajname'] = str(trajname)
			data['est_traj'] = str(est_traj)
			data['ref_traj'] = str(ref_traj)
			data['des_traj'] = str(des_traj)
			data['avgerr'] = avgerr
			data['has_dnn'] = True
			data['phys_range'] = str(phys_range)
	else:
		query = ('''select traj_id, trajname, est_traj, ref_traj, des_traj, avgerr, phys_range from DSLPath_res_nd
			where traj_id in (
			select MAX(traj_id) from DSLPath_res_nd); ''')
		cursor.execute(query)

		for traj_id, trajname, est_traj, ref_traj, des_traj, avgerr, phys_range in cursor:
			data['traj_id'] = traj_id
			data['trajname'] = str(trajname)
			data['est_traj'] = str(est_traj)
			data['ref_traj'] = str(ref_traj)
			data['des_traj'] = str(des_traj)
			data['avgerr'] = avgerr
			data['has_dnn'] = True
			data['phys_range'] = str(phys_range)
	cursor.close()
	conn.close()
	return json.dumps(data)

@app.route("/submit_result", methods=['POST'])
def onResultSubmit():
	traj_id = request.args.get('traj_id')
	trajname = request.args.get('trajname')
	est = request.args.get('est_traj')
	ref = request.args.get('ref_traj')
	des = request.args.get('des_traj')
	avg_err = request.args.get('avgerr')
	phys_range = request.args.get('phys_range')
	has_dnn = request.args.get('has_dnn')

	conn = mysql.connect()
	cursor = conn.cursor()
	if has_dnn == 'true' or has_dnn == 'True' or has_dnn == True:
		query = '''INSERT into DSLPath_res_dnn (traj_id, trajname, est_traj, ref_traj, des_traj, avgerr, phys_range)
			VALUES (%d, %s, %s, %s, %s, %f, %s);
		''' %(traj_id, my_str(trajname), my_str(est_traj), my_str(ref_traj), my_str(des_traj), avgerr, my_str(phys_range))
	else:
		query = '''INSERT into DSLPath_res_nd (traj_id, trajname, est_traj, ref_traj, des_traj, avgerr, phys_range)
			VALUES (%d, %s, %s, %s, %s, %f, %s);
		''' %(traj_id, my_str(trajname), my_str(est_traj), my_str(ref_traj), my_str(des_traj), avgerr, my_str(phys_range))
	cursor.execute(query)
	conn.commit()
	cursor.close()
	conn.close()

@app.route("/get_all_trajectories", methods=['GET'])
def get_all_trajs():
	conn = mysql.connect()
	cursor = conn.cursor()
	query = ''' select traj_id, trajname, destraj, orig_range from DSLPath_tbl where traj_id>0;'''
	cursor.execute(query)
	data = []
	for (traj_id, trajname, destraj, orig_range) in cursor:
		d = {}
		d['traj_id'] = traj_id
		d['trajname'] = str(trajname)
		d['destraj'] = str(destraj)
		d['orig_range'] = str(orig_range)
		data.append(d)

	cursor.close()
	conn.close()
	return json.dumps(data)

@app.route("/getTrajDetailById", methods=['GET'])
def get_details_by_id():
	traj_id = str(request.args.get('traj_id'))
	data = {}
	conn = mysql.connect()
	cursor = conn.cursor()
	query = '''select traj_id, trajname, est_traj, des_traj, ref_traj, avgerr, phys_range from DSLPath_res_nd where traj_id = %s;''' %traj_id
	cursor.execute(query)
	for (traj_id, trajname, est_traj, des_traj, ref_traj, avgerr, phys_range) in cursor:
		#data['nodnn'] = (traj_id, str(trajname), str(est_traj), str(des_traj), str(ref_traj), avgerr, str(phys_range))
		data['nodnn'] = {
			"traj_id": traj_id,
			"trajname": str(trajname),
			"est_traj": str(est_traj),
			"des_traj": str(des_traj),
			"ref_traj": str(ref_traj),
			"avgerr": avgerr,
			"phys_range": str(phys_range)
			}

	query = '''select traj_id, trajname, est_traj, des_traj, ref_traj, avgerr, phys_range from DSLPath_res_dnn where traj_id = %s;''' %traj_id
	cursor.execute(query)
	for (traj_id, trajname, est_traj, des_traj, ref_traj, avgerr, phys_range) in cursor:
		data['hasdnn'] = {
			"traj_id": traj_id,
			"trajname": str(trajname),
			"est_traj": str(est_traj),
			"des_traj": str(des_traj),
			"ref_traj": str(ref_traj),
			"avgerr": avgerr,
			"phys_range": str(phys_range)
			}
	return json.dumps(data)

if __name__ == "__main__":
	app.run()
