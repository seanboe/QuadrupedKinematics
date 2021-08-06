# Gait Creator !

# This is outdated. Fix later

step_user_input = "s"
draw_back_user_input = "db"
shift_user_input = "sft"

gait_parameters = []
gait_schedule = []

def get_gait_code(leg_action):
  gait_scheduler_codes = open('lib/QuadrupedKinematics/src/gait-scheduler-codes.h', 'r')
  lines = gait_scheduler_codes.readlines()

  content_line = 0
  while lines[content_line] != "// --content--\n"  :
    content_line += 1
    if content_line > 50:
      print("What did you do to the content in gait-scheduler-codes.h?")
      print("Quiting")
      exit()


  step_code = int(lines[content_line + 1][lines[content_line + 1].find("1"): ].strip("\n"))
  draw_back_code = int(lines[content_line + 2][lines[content_line + 2].find("1"): ].strip("\n"))
  shift_code = int(lines[content_line + 3][lines[content_line + 3].find("1"): ].strip("\n"))
  minimum_pause = int(lines[content_line + 4][lines[content_line + 4].find("1"): ].strip("\n")) - 10

  if leg_action == step_user_input:
    return step_code
  elif leg_action == draw_back_user_input:
    return draw_back_code
  elif leg_action == shift_user_input:
    return shift_code
  elif leg_action == "minimum_pause":
    return minimum_pause
  


def print_user_codes():
  print(f"Select an action (step = '{step_user_input}', draw back = '{draw_back_user_input}', shift = '{shift_user_input}') for each of the 4 legs for every step.")

def make_multiple_of(number, multiple):
  if (number % multiple > (multiple / 2)):
    return (number // multiple) * multiple + multiple
  else:
    return (number // multiple) * multiple


def formatArray(array):
  array = str(array)
  array = array.replace("[", "{")
  array = array.replace("]", "}")
  return array

def main():
  
  # Establishing the front matter (the basic parameters of the gait)
  stride_length = int(input("Enter the stride length "))
  step_amplitude = int(input("Enter the gait amplitude "))
  draw_back_reduction = int(input("Enter the multiplication factor of the amplitude during the draw back phase"))
  step_duration = int(input("Enter the duration of one step in milliseconds "))
  gait_step_count = int(input("Will your gait have a two step cycle or a four step cycle? (2/4) "))
  gait_duration = int(input("Enter the duration of a step pause in milliseconds (if you don't want one, enter '0'). \n")) + get_gait_code("minimum_pause")

  if (gait_step_count == 4):
    gait_parameters.append(make_multiple_of(stride_length, 3))
  else:
    gait_parameters.append(stride_length)
  gait_parameters.append(step_amplitude)
  gait_parameters.append(draw_back_reduction)
  gait_parameters.append(step_duration)
  gait_parameters.append(gait_step_count)
  gait_parameters.append(gait_duration)

  print(f"Time to create the schedule! The wizard will allow you to select an action (step = '{step_user_input}', draw back = '{draw_back_user_input}', shift = '{shift_user_input}') for each of the 4 legs for every step.")
  print("Please note that the shift is an action for ALL the legs that is applied to the succeeding step. It occurs during a step.")
  print("If you forget a code, enter 'help' on any stage of the wizard.\n")

  # Enter in the gait
  # for step_number in range(0, gait_step_count):
  schedule_finished = False
  step_number = 1
  schedule_actions = 0
  while schedule_finished == False:

    schedule_actions += 1

    step = []

    print(f"In step {step_number}")

    for leg in range(0,4):
      leg_action_input = input(f"Enter the action for leg {leg + 1}\n").lower()

      if leg_action_input == "help":
        print_user_codes()
        leg_action_input = input(f"Enter the action for leg {leg}\n").lower()
        step_number -= 1
        break

      if leg_action_input == "done":
        schedule_finished = True
        schedule_actions -= 1
        break

      elif leg_action_input == step_user_input or leg_action_input == draw_back_user_input:
        step.append(get_gait_code(leg_action_input))

      elif leg_action_input == shift_user_input:
        if leg == 0:
          step.append(get_gait_code(leg_action_input))
          step.append(int(input("What is the x-axis offset?")))
          step.append(int(input("What is the y-axis offset?")))
          step_number -= 1
          break
        else:
          print("Error: shift commands must be given on the first step.")
          print("Quiting")
          exit()

      else:
        print(f"Error: {leg_action_input} is not a valid character.")
        print("Quiting")
        exit()

    if not schedule_finished:
      gait_schedule.append(step)

    if step_number == gait_step_count + 1:
      break

    step_number += 1

  gait_parameters.append(schedule_actions)

  print("Gait Parameters:", formatArray(gait_parameters))
  print("Gait Schedule:", formatArray(gait_schedule))
  print()

if __name__ == "__main__":
  main()