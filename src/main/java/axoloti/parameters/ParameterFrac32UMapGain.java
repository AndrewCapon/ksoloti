/**
 * Copyright (C) 2013, 2014 Johannes Taelman
 * Edited 2023 - 2024 by Ksoloti
 *
 * This file is part of Axoloti.
 *
 * Axoloti is free software: you can redistribute it and/or modify it under the
 * terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * Axoloti is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE. See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * Axoloti. If not, see <http://www.gnu.org/licenses/>.
 */
package axoloti.parameters;

import axoloti.datatypes.ValueFrac32;
import axoloti.realunits.LinDB;
import axoloti.realunits.LinRatio;
import axoloti.realunits.NativeToReal;

/**
 *
 * @author Johannes Taelman
 */
public class ParameterFrac32UMapGain extends ParameterFrac32UMap {

    public ParameterFrac32UMapGain() {
        super();
    }

    public ParameterFrac32UMapGain(String name) {
        super(name);
    }

    public ParameterFrac32UMapGain(String name, ValueFrac32 DefaultValue) {
        super(name, DefaultValue);
    }

    @Override
    public ParameterInstanceFrac32UMap InstanceFactory() {
        ParameterInstanceFrac32UMap p = super.InstanceFactory();
        NativeToReal convs[] = {new LinRatio(1.0), new LinDB(0.0)};
        p.SetPFunction("pfun_unsigned_clamp_fullrange");
        p.convs = convs;
        return p;
    }

    static public final String TypeName = "frac32.u.map.gain";

    @Override
    public String getTypeName() {
        return TypeName;
    }
}
