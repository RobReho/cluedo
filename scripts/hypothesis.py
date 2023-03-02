from armor_librery import *
import rospy
import random

def main():
    rospy.init_node('ontology_client')
    armor = Armor_Communication()
    armor.__init__()
    armor.log_to_file()
    armor.load_file()
    
   
    pl = armor.retrieve_class('PLACE')
    place = random.choice(pl)
    
    pe = armor.retrieve_class('PERSON')
    person = random.choice(pe)
    
    we = armor.retrieve_class('WEAPON')
    weapon = random.choice(we)
    
    armor.remove_instance(weapon,'WEAPON')
    armor.reason()
    print(armor.retrieve_class('WEAPON'))
    
    armor.make_hypothesis(person,weapon,place)
    armor.reason()
    
    
    
    
    
    #print( armor.retrieve_class('COMPLETED'))
    
    
    rospy.sleep(1)
    



if __name__ == "__main__":
    main()
 
