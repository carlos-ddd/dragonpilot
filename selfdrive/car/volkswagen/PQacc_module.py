import numpy
import math

from selfdrive.car.volkswagen.values import BUTTON_STATES

class PQacc():
    """
    """
#    BUTTON_STATES = {
#      "accelCruise": False,
#      "decelCruise": False,
#      "cancel": False,
#      "setCruise": False,
#      "resumeCruise": False,
#      "gapAdjustCruise": False,
#      "longUp" : False,
#      "longDown" : False
#    }

    BUTTON_ACTIONS = {
            "undefined" : "?",
            "noPress"   : "_",
            "pressed"   : "=",
            "rising"    : "^",
            "falling"   : "v"
    }

    def __init__(self, v_min=1., v_max=210., v_step=10., v_lst=[125], dist_maxval=2 , dist_long_maxval=2 , longpress_timer=100 , engage_below_vmin=False, round_at_set=True):
        self.v_min = v_min
        self.v_max = v_max
        self.v_step = v_step
        self.v_lst = v_lst
        self.engage_below_vmin = engage_below_vmin
        self.round_at_set = round_at_set
        self.dist_longpress_timer = longpress_timer # counting calls of function, not time
        self.dist_maxval = dist_maxval # maximum value of distance (between 0 and this value)
        self.dist_long_maxval = dist_long_maxval # maximum value of distance longpress (between 0 and this value)
        self.assign_default_vals()

    def assign_default_vals(self):
        self.v_set = float('NaN')
        self.longCtrl_engaged = False
        self.longCtrl_coast = False
        self.v_mem = float('NaN')
        self.dist = 0
        self.dist_long = 0
        self.dist_val_raw_old = 0
        self.buttonStatesOld = BUTTON_STATES.copy()
        self.buttonActionsOld = BUTTON_STATES.copy()
        self.v_disp = float('NaN')
        self.op_engagedOld = False
        self.dist_longpress_timer_val = 0

    def inp_mainSW(self, GRA_Haupt):
        if not GRA_Haupt:                                           # reinit with default values
            self.assign_default_vals()

    def get_vset_valid(self):
        return not numpy.isnan(self.v_set)

    def get_vdisp_valid(self):
        return not numpy.isnan(self.v_disp)


# --- Testbench ------------------------------------------------------------------------------------------------------

    def get_forTest(self):
        return self.longCtrl_engaged, self.v_set, self.dist, self.longCtrl_coast, self.v_mem, self.v_disp, self.get_vset_valid(), self.get_vdisp_valid()

    def prnt_outSta(self):
        print('->',("%.1f"%(self.v_set)).ljust(5),'(',('X' if self.longCtrl_engaged else ' '),('C' if self.longCtrl_coast else ' '),')','mem:',("%.1f"%(self.v_mem)).ljust(5),'disp:',("%.1f"%(self.v_disp)).ljust(5),'dist:',int(self.dist))

    def prnt_btnX(self, btnX, prntKeysOff):
        if not prntKeysOff: 
            for _key, _val in btnX.items():
                print(' |',str(_key).ljust(max(len(str(_key)),len(str(_val)))),end='')
            print(' |')
        for _key, _val in btnX.items():
            print(' |',str(_val).center(max(len(str(_key)),len(str(_val)))),end='')
        print(' |')

    def prnt_btnState(self, btnSta, prntKeysOff):
        self.prnt_btnX(btnSta, prntKeysOff)

    def prnt_btnAction(self, btnAct, prntKeysOff):
        self.prnt_btnX(btnAct, prntKeysOff)

    def prnt_btnActionShort(self, btnAct, inhibitNewline, prntKeysOff=True):

        if not prntKeysOff:
            print('+----------------- accelCruise')
            print('| +--------------- decelCruise')
            print('| | +------------- cancel')
            print('| | | +----------- setCruise')
            print('| | | | +--------- resumeCruise')
            print('| | | | | +------- gapAdjustCruise')
            print('| | | | | | +----- longUp')
            print('| | | | | | | +--- longDown')
            print('| | | | | | | |')
        print('| ', end='')
        for _key, _val in btnAct.items():
            print(_val,'',end='')
        if inhibitNewline:
            print("| ", end='')
        else:
            print("|")

# --- end of: Testbench -----------------------------------------------------------------------------------------------------

    def get_btnActions(self, btnSta_old, btnSta_new):
        '''
        button states to button actions
        '''
        btnAct = BUTTON_STATES.copy()                          # return
        for key, val in btnSta_new.items():
            if val == True and btnSta_old[key] == False:
                btnAct[key] = self.BUTTON_ACTIONS['rising']
            if val == False and btnSta_old[key] == True:
                btnAct[key] = self.BUTTON_ACTIONS['falling']
            elif val == btnSta_old[key] == False:
                btnAct[key] = self.BUTTON_ACTIONS['noPress']
            elif val == btnSta_old[key] == True:
                btnAct[key] = self.BUTTON_ACTIONS['pressed']
        return btnAct


    def update_acc(self, v_Ego, btnAct_old, btnAct_new, btnSta_new, gap_adjust_raw, GRA_Haupt, op_disengageTrg, stockGRA_active=False):
        '''
        takes button actions and dispatches them to the appropriate handling functions
        - priority is first decided here by if-elseif-else
        '''

        if numpy.isnan(v_Ego) or v_Ego<0:
            v_Ego = 0.

        if GRA_Haupt:
            if op_disengageTrg:
                self._regular_disengage()
            else:
                # hint: immedate buttonpress after cancle will be ignored since it's depatched to CANCEL-button handling
                if btnAct_old['cancel'] != btnAct_new['cancel']:
                    if not stockGRA_active: # in case of the stock VW GRA not being dropped yet -> allow for drop until OPacc evaluates cancle-button-presses
                        self.handle_cancelBtn(btnAct_new)
                else:
                    self.handle_speedBtn(v_Ego, btnAct_new)
            # allow always (as long as GRA-Haupt is not off)
            self.handle_distBtn(btnAct_new, btnSta_new, gap_adjust_raw) # alternativ (old) only on change elif btnAct_old['gapAdjustCruise'] != btnAct_new['gapAdjustCruise']:
        else:   # after GRA off -> default values
            self.assign_default_vals()

       # btn             falling     rising    hold
       # accelCruise                   X
       # decelCruise                   X
       # cancel                        X
       # setCruise                     X
       # resumeCruise                  X
       # longUp                        X            (automatically set after ~500ms holding and falling with accelCruise)
       # longDown          X           X       X    (automatically set after ~500ms holding and falling with decelCruise)


    def update_acc_iter(self, v_Ego, btnSta_new, gap_adjust_raw, GRA_Haupt, op_engaged, stockGRA_active=False):
        '''
        Takes new button states and calls appropriate actions to evaluate
        then saves old button states for next step
        '''
        op_disengageTrg = True if (op_engaged==False and self.op_engagedOld==True) else False

        btnAct_new = self.get_btnActions(self.buttonStatesOld, btnSta_new)                      # calculate button actions
#        self.prnt_btnActionShort(btnAct_new, True, True)
        self.update_acc(v_Ego, self.buttonActionsOld, btnAct_new, btnSta_new, gap_adjust_raw, GRA_Haupt, op_disengageTrg, stockGRA_active)   # pass button actions

        # save old states
        self.buttonStatesOld = btnSta_new.copy()
        self.buttonActionsOld = btnAct_new.copy()
        self.op_engagedOld = op_engaged

    def update_acc_iter_4CS(self, v_Ego, btnSta_new, gap_adjust_raw, GRA_Haupt, op_engaged, stockGRA_active=False):
        '''
        workaround wrapper function to use OPacc directly in carstate.py
        this function returns desired engagement status of OP (for engage / disengage)
        and a always valid setspeed for OP (as opposed to update_acc_iter() / update_acc()
        which will turn setspeed to 'NaN' if not engaged
        '''
        self.update_acc_iter(v_Ego, btnSta_new, gap_adjust_raw, GRA_Haupt, op_engaged, stockGRA_active)
        v_4CS = self.v_set if self.get_vset_valid() else (self.v_disp if self.get_vdisp_valid() else 88.)
        return self.longCtrl_engaged, v_4CS, self.dist, self.dist_long

    def handle_speedBtn(self, v_Ego, btnAct):
        '''
        handles speed manipulating button presses
        '''
        # priority defined by order / elseif !

        # if engaged use v_set, if not use v_mem (only used with +/-10 action)
        v_temp_in = self.v_set if self.longCtrl_engaged else self.v_mem
        v_temp_out = float('NaN')

        if btnAct['setCruise']==self.BUTTON_ACTIONS['rising']:
            if self.longCtrl_engaged:
                self.v_set -= 1.                                            # -1
            else:                                                           # set
                if v_Ego >= self.v_min or self.engage_below_vmin:
                    self.v_set = self._find_intelligentSetSpeed(v_Ego, round_do=self.round_at_set)    # intelligent rounding
                    self.longCtrl_engaged = True
                    self.longCtrl_coast = False

        elif btnAct['resumeCruise']==self.BUTTON_ACTIONS['rising']:
            if self.longCtrl_engaged:
                self.v_set += 1.                                            # +1
            else:
                if v_Ego >= self.v_min or self.engage_below_vmin:
                    if numpy.isnan(self.v_mem):                             # no value to recall -> set
                        self.v_set = self._find_intelligentSetSpeed(v_Ego, round_do=self.round_at_set)    # intelligent rounding
                    else:
                        self.v_set = self.v_mem                             # recall
                    self.longCtrl_engaged = True
                    self.longCtrl_coast = False

        elif btnAct['longDown']==self.BUTTON_ACTIONS['rising'] or btnAct['longDown']==self.BUTTON_ACTIONS['pressed']:
            if self.longCtrl_engaged:                                       # v_set = v_Ego -> no brakes / gas applied (automatically) = coasting
                self.v_set = v_Ego
                self.longCtrl_coast = True
        elif btnAct['longDown']==self.BUTTON_ACTIONS['falling']:
            if self.longCtrl_engaged:                                       # coasting done
                self.longCtrl_coast = False

        elif btnAct['accelCruise']==self.BUTTON_ACTIONS['rising']:          # allow +/-10 in disengaged
            v_temp_out = self._find_nextSpeed_up(v_temp_in, self.v_step, self.v_lst)

        elif btnAct['decelCruise']==self.BUTTON_ACTIONS['rising']:
            v_temp_out = self._find_nextSpeed_down(v_temp_in, self.v_step, self.v_lst)

        # assign depending on engage state
        if not numpy.isnan(v_temp_out):
            if self.longCtrl_engaged:
                self.v_set = v_temp_out
            else:
                self.v_mem = v_temp_out

        # clip values to v_min .. v_max
        if not numpy.isnan(self.v_set):
            self.v_set = self.v_set if self.v_set <= self.v_max else self.v_max
            self.v_set = self.v_set if self.v_set >= self.v_min else self.v_min
        if not numpy.isnan(self.v_mem):
            self.v_mem = self.v_mem if self.v_mem <= self.v_max else self.v_max
            self.v_mem = self.v_mem if self.v_mem >= self.v_min else self.v_min

        # select displayed speed
        self.v_disp = self.v_set if not numpy.isnan(self.v_set) else self.v_mem

    def _find_nextSpeed_up(self, v_in, v_step, v_lst):
        '''
        returns next higher speed (either speed-step or from speed-list)
        '''
        v_next_step = self._find_nextStepSpeed_up(v_in, v_step)
        lst_found, v_next_lst = self._find_nextLstSpeed_up(v_in, v_lst)

        v_out = min(v_next_step, v_next_lst) if lst_found else v_next_step
        return v_out

    def _find_nextSpeed_down(self, v_in, v_step, v_lst):
        '''
        returns next lower speed (either speed-step or from speed-list)
        '''
        v_next_step = self._find_nextStepSpeed_down(v_in, v_step)
        lst_found, v_next_lst = self._find_nextLstSpeed_down(v_in, v_lst)

        v_out = max(v_next_step, v_next_lst) if lst_found else v_next_step
        return v_out

    def _find_nextLstSpeed_up(self, v_in, v_lst):
        '''
        returns next higher speed from speed-list
        returns input-speed if non found
        '''
        v_out = v_in                                                    # default
        found_in_lst = False

        if len(v_lst) > 0:
            for v in v_lst:
                if v > v_in:
                    v_out = v
                    found_in_lst = True
                    break;
        return found_in_lst, v_out

    def _find_nextLstSpeed_down(self, v_in, v_lst):
        '''
        returns next lower speed from speed-list
        returns input-speed if non found
        '''
        v_out = v_in                                                    # default
        found_in_lst = False
        v_lst_cpy = v_lst.copy()                                        # list.reverse() is a in-place-opertation
                                                                        #  -> a copy of the list is needed !
        if len(v_lst_cpy) > 0:
            v_lst_cpy.reverse()
            for v in v_lst_cpy:
                if v < v_in:
                    v_out = v
                    found_in_lst = True
                    break;
        return found_in_lst, v_out

    def _find_nextStepSpeed_up(self, v_in, v_step):
        '''
        returns next hight speed based on speed-step
        '''
        v_out = float('NaN')
        if v_in % v_step == 0.:
            v_out = v_in + v_step
        else:
            v_out = float(int(math.ceil(v_in / v_step)) * int(v_step))   # set to nearest +10*km/h
        return v_out

    def _find_nextStepSpeed_down(self, v_in, v_step):
        '''
        returns next lower speed based on speed-step
        '''
        v_out = float('NaN')
        if v_in % v_step == 0.:
            v_out = v_in - v_step
        else:
            v_out =float(int(math.floor(v_in / v_step)) * v_step)       # set to nearest +10*km/h
        return v_out

    def _find_intelligentSetSpeed(self, v_Ego, round_do=True, round_intelligent=True):
        if round_do:
            if round_intelligent:
                v_Ego_Tens = int(v_Ego/10)*10
                v_Ego_Remain = v_Ego%10
                if v_Ego_Remain > 3.0:                          # round up unconventionally (larger than 3.0)
                    retVal = v_Ego_Tens + 10
                else:
                    retVal = v_Ego_Tens
            else:
                retVal = round(v_Ego/10)*10                     # round (mathematically) to next 10kmh
        else:
            retVal = v_Ego                                      # do not round -> just use v_Ego
        return retVal

    def handle_cancelBtn(self, btnAct):
        '''
        handles cancel button press
        '''
        if self.longCtrl_engaged:
            # cancle release -> disengage
            if btnAct['cancel'] == self.BUTTON_ACTIONS['rising']:
                self._regular_disengage()

    def _regular_disengage(self):
        '''
        triggers normal disengage
        '''
        self.v_mem = self.v_set     # save current setpoint for resume
        self.longCtrl_engaged = False
        self.longCtrl_coast = False
        self.v_set = float('NaN')


    def handle_distBtn(self, btnAct, btnSta_new, gap_adjust_raw):
        '''
        handles distance button
        '''
        retVal = float('NaN')
        dist_val_raw = gap_adjust_raw
        if (dist_val_raw == 3) or numpy.isnan(self.dist_val_raw_old):
            # error, no evaluation possible
            self.dist = 0 #float('NaN')
            self.dist_long = 0 #float('NaN')
        else:
            if (self.dist_val_raw_old == 0) and (dist_val_raw == 1):
                # rising Dist-
                self.dist_longpress_timer_val = 0
            elif (self.dist_val_raw_old == 0) and (dist_val_raw == 2):
                # rising Dist+
                self.dist_longpress_timer_val = 0
            elif (self.dist_val_raw_old == 1) and (dist_val_raw == 1):
                # high Dist-
                self.dist_longpress_timer_val += 1
            elif (self.dist_val_raw_old == 2) and (dist_val_raw == 2):
                # high Dist+
                self.dist_longpress_timer_val += 1
            elif (self.dist_val_raw_old == 1) and (dist_val_raw == 0):
                # ==============
                # falling Dist-
                # ==============
                if self.dist_longpress_timer_val > self.dist_longpress_timer:
                    # -------------------
                    # longpress detected
                    # -------------------
                    if self.dist_long - 1 < 0:
                        self.dist_long = 0
                    else:
                        self.dist_long -= 1
                else:
                    # -------------------
                    # shortpress detected
                    # -------------------
                    if self.dist - 1 < 0:
                        self.dist = 0
                    else:
                        self.dist -= 1
                self.dist_longpress_timer_val = 0
            elif (self.dist_val_raw_old == 2) and (dist_val_raw == 0):
                # ==============
                # falling Dist+
                # ==============
                if self.dist_longpress_timer_val > self.dist_longpress_timer:
                    # -------------------
                    # longpress detected
                    # -------------------
                    if self.dist_long + 1 > self.dist_long_maxval:
                        self.dist_long = self.dist_long_maxval
                    else:
                        self.dist_long += 1
                else:
                    # -------------------
                    # shortpress detected
                    # -------------------
                    if self.dist + 1 > self.dist_maxval:
                        self.dist = self.dist_maxval
                    else:
                        self.dist += 1
                self.dist_longpress_timer_val = 0
            else:
                self.dist_longpress_timer_val = 0   # reset
        self.dist_val_raw_old = dist_val_raw    # update for next iteration
        print(self.dist_longpress_timer_val)
        return self.dist, self.dist_long


