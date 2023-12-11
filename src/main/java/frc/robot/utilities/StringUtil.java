/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.utilities;

@SuppressWarnings("MemberName")
public class StringUtil {

    /**
     * Uses StringBuilder to concatenate the elements of the parameter array
     * @param oarray any number of comma-separated objects to be put into an array 
     * @return a String made of oarray
     */
	public static String buildString(Object... oarray) {
		StringBuilder s = new StringBuilder(1024);

		for (int i = 0; i<oarray.length; i++) {
			s = s.append( oarray[i].toString() );
		}
		return s.toString();
    }
    
    /**
     * Uses StringBuilder to concatenate the elements of the parameter array
     * Adds commas between elements (no comma before the first element or after the last element)
     * @param oarray any number of comma-separated objects to be put into an array 
     * @return a String made of oarray with commas between the elements
     */
    public static String buildStringWithCommas(Object... oarray) {
        StringBuilder s = new StringBuilder(1024);

		for (int i = 0; i<oarray.length; i++) {
            s = s.append( oarray[i].toString() );
            if (i < (oarray.length - 1)) {
                s = s.append(",");
            }
		}
		return s.toString();
    }
}
