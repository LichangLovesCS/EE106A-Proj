def One_Turn(self,target_marker,radian):
	    def find_location(AR_marker,target_marker,Handness):
	        if Handness == 'left':
	            if AR_marker == 'ar_marker_1':
	                m1 = {'ar_marker_2':'upper_left','ar_marker_5':'upper_right',
	                        'ar_marker_3':'front', 'ar_marker_4':'back',
	                        'ar_marker_1':'lower_left','ar_marker_6':'lower_right'}
	            elif AR_marker == 'ar_marker_2':
	                m1 = {'ar_marker_5':'upper_left','ar_marker_6':'upper_right',
	                        'ar_marker_3':'front', 'ar_marker_4':'back',
	                        'ar_marker_2':'lower_left','ar_marker_1':'lower_right'}
	            elif AR_marker == 'ar_marker_3':
	                m1 = {'ar_marker_6':'upper_left','ar_marker_4':'upper_right',
	                        'ar_marker_1':'front', 'ar_marker_5':'back',
	                        'ar_marker_3':'lower_left','ar_marker_2':'lower_right'}
	            elif AR_marker == 'ar_marker_4':
	                m1 = {'ar_marker_2':'upper_left','ar_marker_3':'upper_right',
	                        'ar_marker_1':'front', 'ar_marker_5':'back',
	                        'ar_marker_4':'lower_left','ar_marker_6':'lower_right'}
	            elif AR_marker == 'ar_marker_5':
	                m1 = {'ar_marker_2':'upper_left','ar_marker_4':'upper_right',
	                        'ar_marker_5':'front', 'ar_marker_1':'back',
	                        'ar_marker_3':'lower_left','ar_marker_6':'lower_right'}
	            elif AR_marker == 'ar_marker_6':
	                m1 = {'ar_marker_1':'upper_left','ar_marker_2':'upper_right',
	                        'ar_marker_3':'front', 'ar_marker_4':'back',
	                        'ar_marker_6':'lower_left','ar_marker_5':'lower_right'}
	        elif Handness == 'right':
	            if AR_marker == 'ar_marker_1':
	                m1 = {'ar_marker_6':'upper_left','ar_marker_4':'upper_right',
	                        'ar_marker_1':'front', 'ar_marker_5':'back',
	                        'ar_marker_3':'lower_left','ar_marker_2':'lower_right'}
	            elif AR_marker == 'ar_marker_2':
	                m1 = {'ar_marker_2':'upper_left','ar_marker_4':'upper_right',
	                        'ar_marker_5':'front', 'ar_marker_1':'back',
	                        'ar_marker_3':'lower_left','ar_marker_6':'lower_right'}
	            elif AR_marker == 'ar_marker_3':
	                m1 = {'ar_marker_2':'upper_left','ar_marker_4':'upper_right',
	                        'ar_marker_5':'front', 'ar_marker_1':'back',
	                        'ar_marker_3':'lower_left','ar_marker_6':'lower_right'}
	            elif AR_marker == 'ar_marker_4':
	                m1 = {'ar_marker_1':'upper_left','ar_marker_2':'upper_right',
	                        'ar_marker_3':'front', 'ar_marker_4':'back',
	                        'ar_marker_6':'lower_left','ar_marker_5':'lower_right'}
	            elif AR_marker == 'ar_marker_5':
	                m1 = {'ar_marker_1':'upper_left','ar_marker_2':'upper_right',
	                        'ar_marker_3':'front', 'ar_marker_4':'back',
	                        'ar_marker_6':'lower_left','ar_marker_5':'lower_right'}
	            elif AR_marker == 'ar_marker_6':
	                m1 = {'ar_marker_2':'upper_left','ar_marker_5':'upper_right',
	                        'ar_marker_3':'front', 'ar_marker_4':'back',
	                        'ar_marker_1':'lower_left','ar_marker_6':'lower_right'}
	        return m1
	    def Get_lower_right(position):
	        key_list=[]  
	        value_list=[]  
	        for key,value in position.items():  
	            key_list.append(key)  
	            value_list.append(value)   
	        get_value_index = value_list.index('lower_right')  
	        return key_list[get_value_index]  
	    def Get_lower_left(position):
	        key_list=[]  
	        value_list=[]  
	        for key,value in position.items():  
	            key_list.append(key)  
	            value_list.append(value)   
	        get_value_index = value_list.index('lower_left')  
	        return key_list[get_value_index]

	    if self.Holder_Arm.name =='left':
	        left_marker = self.Get_Marker('left')
	        self.Handness = 'left'
	        position = find_location(left_marker,target_marker,'left')
	        self.Leave_Cube('right')
	        #find the motion state
	        if tar_position =='lower_left':
	            low_right_marker = Get_lower_right(position)
	            self.Grab_Cube('right',low_right_marker)
	            self.Rotation('left',radian)
	            self.Leave_Cube('right')
	        elif tar_position =='lower_right':
	            self.Grab_Cube('right',target_marker)
	            self.Rotation('right',radian)
	            self.Leave_Cube('right')
	        elif tar_position =='upper_left':
	            self.Rotation('left',pi)
	            self.Grab_Cube('right',target_marker)
	            self.Leave_Cube('left')
	            #There`s a bug
	            low_left_marker = Get_lower_right(position)
	            self.Grab_Cube('right',lower_left_marker)
	            self.Rotation('right',radian)
	        elif tar_position =='upper_right':
	            low_right_marker = Get_lower_right(position)
	            self.Grab_Cube('right',low_right_marker)
	            self.Leave_Cube('left')
	            self.Rotation('right',pi)
	            self.Grab_Cube('left',target_position)
	            self.Rotation('left',radian)
	            self.Leave_Cube('right')
	        elif tar_position == 'front':
	            self.Rotation('left',pi/2)
	            #Here has a bug
	            self.Rotation('right',radian)
	            low_left_marker = Get_Marker('left')
	            self.Leave_Cube('left')
	            self.Grab_Cube('left',low_left_marker)
	            self.Leave_Cube('right')
	        elif tar_position =='back':
	            self.Rotation('left',(3*pi)/2)
	            self.Rotation('right',radian)
	            low_left_marker = Get_Marker('left')
	            self.Leave_Cube('left')
	            self.Grab_Cube('left',low_left_marker)
	            self.Leave_Cube('right')
	        else:
	            print('Invalid target_marker in One_Turn function.')
	    elif self.Holder_Arm.name == 'right':
	        right_makrer = self.Get_Marker('right')
	        self.Handness = 'right'
	        position = find_location(right_makrer,target_marker,'right')
	        self.Leave_Cube('left')
	        if tar_position =='lower_right':
	            low_left_marker = Get_lower_left(position)
	            self.Grab_Cube('left',low_left_marker)
	            self.ROtation('right',radian)
	            self.Leave_Cube('left')
	        elif tar_position == 'lower_left':
	            self.Grab_Cube('left',target_marker)
	            self.Rotation('right',radian)
	            self.Leave_Cube('left')
	        elif tar_position == 'upper_right':
	            self.Rotation('right',pi)
	            self.Grab_Cube('left',target_marker)
	            self.Leave_Cube('right')
	            #There`s must exist a bug
	            low_right_marker = Get_lower_left(position)
	            self.Grab_Cube('left',low_right_marker)
	            self.Rotation('left',radian)
	        elif tar_position =='upper_left':
	            low_left_marker = Get_lower_left(position)
	            self.Grab_Cube('left',low_left_marker)
	            self.Leave_Cube('right')
	            self.Rotation('left',pi)
	            self.Grab_Cube('right',target_position)
	            self.Rotation('right',radian)
	            self.Leave_Cube('left')
	        elif tar_position=='front':
	            self.Rotation('right',pi/2)
	            self.Rotation('left',radian)
	            low_left_marker = Get_Marker('right')
	            self.Leave_Cube('right')
	            self.Grab_Cube('right',low_right_marker)
	            self.Leave_Cube('left')
	        elif tar_position=='back':
	            self.Rotation('right',(3*pi)/2)
	            self.Rotation('left',radian)
	            low_right_marker = Get_Marker('right')
	            self.Leave_Cube('right')
	            self.Grab_Cube('right',low_right_marker)
	            self.Leave_Cube('left')