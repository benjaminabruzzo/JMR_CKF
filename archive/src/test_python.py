import yaml


with open('/home/benjamin/ros/src/metahast/hast_gazebo/config/logger_dictionary.yaml') as f:
	body_labels = yaml.load(f)
	print(body_labels)


ugv1_dict = dict()
ugv2_dict = dict()
for body_label in body_labels:
	print(body_label)
	# if body_label['name'] == "ugv1":
	# 	ugv1['path'] = body_label['path']
	# if body_label['name'] == "ugv2":
	# 	ugv2['path'] = body_label['path']



# data = yaml.load(f, Loader=yaml.FullLoader)
