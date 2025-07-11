/**
 * Copyright (C) 2013 - 2016 Johannes Taelman
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
package axoloti;

/**
 * Replaces the old packet-over-serial protocol with vendor-specific usb bulk
 * transport
 */
import axoloti.dialogs.USBPortSelectionDlg;

import static axoloti.MainFrame.prefs;
import static axoloti.dialogs.USBPortSelectionDlg.ErrorString;
import axoloti.displays.DisplayInstance;
import axoloti.parameters.ParameterInstance;
import axoloti.targetprofile.ksoloti_core;
import java.lang.reflect.InvocationTargetException;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.CharBuffer;
import java.nio.IntBuffer;
import java.nio.charset.Charset;
import java.util.Calendar;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.BlockingQueue;
import java.util.logging.Level;
import java.util.logging.Logger;
import javax.swing.SwingUtilities;
import org.usb4java.*;
import qcmds.QCmd;
import qcmds.QCmdMemRead;
import qcmds.QCmdMemRead1Word;
import qcmds.QCmdProcessor;
import qcmds.QCmdSerialTask;
import qcmds.QCmdSerialTaskNull;
import qcmds.QCmdShowDisconnect;
import qcmds.QCmdTransmitGetFWVersion;

/**
 *
 * @author Johannes Taelman
 */
public class USBBulkConnection extends Connection {

    private static final Logger LOGGER = Logger.getLogger(USBBulkConnection.class.getName());

    private Patch patch;
    private boolean disconnectRequested;
    private boolean connected;
    private Thread transmitterThread;
    private Thread receiverThread;
    private final BlockingQueue<QCmdSerialTask> queueSerialTask;
    private String cpuid;
    private ksoloti_core targetProfile;
    private final Context context;
    private DeviceHandle handle;
    private final short bulkVID = (short) 0x16C0;
    private final short bulkPIDAxoloti = (short) 0x0442;
    private final short bulkPIDAxolotiUsbAudio = (short) 0x0447;
    private final short bulkPIDKsoloti = (short) 0x0444;
    private final short bulkPIDKsolotiUsbAudio = (short) 0x0446;
    private int useBulkInterfaceNumber = 2;

	protected USBBulkConnection() {
        this.sync = new Sync();
        this.readsync = new Sync();
        this.patch = null;

        disconnectRequested = false;
        connected = false;
        queueSerialTask = new ArrayBlockingQueue<QCmdSerialTask>(10);
        context = new Context();

        int result = LibUsb.init(context);
        if (result != LibUsb.SUCCESS) {
            throw new LibUsbException("Unable to initialize libusb.", result);
        }
    }

    @Override
    public void setPatch(Patch patch) {
        this.patch = patch;
    }

    public void Panic() {
        queueSerialTask.clear();
        disconnect();
    }

    @Override
    public boolean isConnected() {
        return connected && (!disconnectRequested);
    }

    @Override
    public boolean AppendToQueue(QCmdSerialTask cmd) {
        return queueSerialTask.add(cmd);
    }

    @Override
    public void disconnect() {
        if (connected) {
            disconnectRequested = true;
            connected = false;
            isSDCardPresent = null;
            ShowDisconnect();
            queueSerialTask.clear();

            try {
                Thread.sleep(100);
            }
            catch (InterruptedException ex) {
                LOGGER.log(Level.SEVERE, null, ex);
            }

            queueSerialTask.add(new QCmdSerialTaskNull());
            queueSerialTask.add(new QCmdSerialTaskNull());

            try {
                Thread.sleep(100);
            }
            catch (InterruptedException ex) {
                LOGGER.log(Level.SEVERE, null, ex);
            }

            LOGGER.log(Level.WARNING, "Disconnected\n");

            synchronized (sync) {
                sync.Acked = false;
                sync.notifyAll();
            }

            if (receiverThread.isAlive()){
                receiverThread.interrupt();

                try {
                    receiverThread.join();
                }
                catch (InterruptedException ex) {
                    LOGGER.log(Level.SEVERE, null, ex);
                }
            }
            
            int result = LibUsb.releaseInterface(handle, useBulkInterfaceNumber);
            if (result != LibUsb.SUCCESS) {
                throw new LibUsbException("Unable to release interface", result);
            }

            LibUsb.close(handle);
            handle = null;
            CpuId0 = 0;
            CpuId1 = 0;
            CpuId2 = 0;
        }
    }

    public DeviceHandle OpenDeviceHandle() {

        /* Read the USB device list */
        DeviceList list = new DeviceList();

        int result = LibUsb.getDeviceList(context, list);
        if (result < 0) {
            throw new LibUsbException("Unable to get device list", result);
        }

        try {
            /* Iterate over all devices and scan for the right one */
            for (Device d : list) {
                DeviceDescriptor descriptor = new DeviceDescriptor();

                result = LibUsb.getDeviceDescriptor(d, descriptor);
                if (result != LibUsb.SUCCESS) {
                    throw new LibUsbException("Unable to read device descriptor", result);
                }

                if (prefs.getFirmwareMode().contains("Ksoloti Core")) {

                    if (descriptor.idVendor() == bulkVID && ((descriptor.idProduct() == bulkPIDKsoloti) || (descriptor.idProduct() == bulkPIDKsolotiUsbAudio))) {

                        if (descriptor.idProduct() == bulkPIDKsolotiUsbAudio) {
                            useBulkInterfaceNumber = 4;
                            LOGGER.log(Level.INFO, "Ksoloti Core USB Audio found.");
                        }
                        else {
                            useBulkInterfaceNumber = 2;
                            LOGGER.log(Level.INFO, "Ksoloti Core found.");
                        }

                        DeviceHandle h = new DeviceHandle();

                        result = LibUsb.open(d, h);
                        if (result < 0) {
                            LOGGER.log(Level.INFO, ErrorString(result));
                        }
                        else {
                            String serial = LibUsb.getStringDescriptor(h, descriptor.iSerialNumber());

                            if (cpuid != null) {

                                if (serial.equals(cpuid)) {
                                    return h;
                                }
                            }
                            else {
                                return h;
                            }

                            LibUsb.close(h);
                        }
                    }
                }
                else if (prefs.getFirmwareMode().contains("Axoloti Core")) {

                    if (descriptor.idVendor() == bulkVID && ((descriptor.idProduct() == bulkPIDAxoloti) || (descriptor.idProduct() == bulkPIDAxolotiUsbAudio))) {

                        if (descriptor.idProduct() == bulkPIDAxolotiUsbAudio) {
                            useBulkInterfaceNumber = 4;
                            LOGGER.log(Level.INFO, "Axoloti Core USB Audio found.");
                        }
                        else {
                            useBulkInterfaceNumber = 2;
                            LOGGER.log(Level.INFO, "Axoloti Core found.");
                        }

                        DeviceHandle h = new DeviceHandle();

                        result = LibUsb.open(d, h);
                        if (result < 0) {
                            LOGGER.log(Level.INFO, ErrorString(result));
                        }
                        else {
                            String serial = LibUsb.getStringDescriptor(h, descriptor.iSerialNumber());

                            if (cpuid != null) {
                                if (serial.equals(cpuid)) {
                                    return h;
                                }
                            }
                            else {
                                return h;
                            }

                            LibUsb.close(h);
                        }
                    }
                }
            }

            /* Or else pick the first one */
            for (Device d : list) {

                DeviceDescriptor descriptor = new DeviceDescriptor();

                result = LibUsb.getDeviceDescriptor(d, descriptor);
                if (result != LibUsb.SUCCESS) {
                    throw new LibUsbException("Unable to read device descriptor", result);
                }

                if (prefs.getFirmwareMode().contains("Ksoloti Core")) {

                    if (descriptor.idVendor() == bulkVID && descriptor.idProduct() == bulkPIDKsoloti) {

                        LOGGER.log(Level.INFO, "Ksoloti Core found.");
                        DeviceHandle h = new DeviceHandle();

                        result = LibUsb.open(d, h);
                        if (result < 0) {
                            LOGGER.log(Level.INFO, ErrorString(result));
                        }
                        else {
                            return h;
                        }
                    }
                }
                else if (prefs.getFirmwareMode().contains("Axoloti Core")) {

                    if (descriptor.idVendor() == bulkVID && descriptor.idProduct() == bulkPIDAxoloti) {

                        LOGGER.log(Level.INFO, "Axoloti Core found.");
                        DeviceHandle h = new DeviceHandle();

                        result = LibUsb.open(d, h);
                        if (result < 0) {
                            LOGGER.log(Level.INFO, ErrorString(result));
                        }
                        else {
                            return h;
                        }
                    }
                }
            }
        }
        finally {
            /* Ensure the allocated device list is freed */
            // LibUsb.freeDeviceList(list, true);
        }

        LOGGER.log(Level.SEVERE, "No matching USB devices found.");

        /* Device not found */
        return null;
    }

    // private byte[] bb2ba(ByteBuffer bb) {
    //     bb.rewind();
    //     byte[] r = new byte[bb.remaining()];
    //     bb.get(r, 0, r.length);
    //     return r;
    // }

    @Override
    public boolean connect() {

        disconnect();
        disconnectRequested = false;

        synchronized (sync) {
            sync.Acked = true;
            sync.notifyAll();
        }

        GoIdleState();

        if (cpuid == null) {
            cpuid = prefs.getComPortName();
        }

        targetProfile = new ksoloti_core();

        handle = OpenDeviceHandle();
        if (handle == null) {
            return false;
        }

        try {
            // devicePath = Usb.DeviceToPath(device);

            int result = LibUsb.claimInterface(handle, useBulkInterfaceNumber);
            if (result != LibUsb.SUCCESS) {
                throw new LibUsbException("Unable to claim interface", result);
            }

            GoIdleState();
            // LOGGER.log(Level.INFO, "Creating rx and tx thread...");
            transmitterThread = new Thread(new Transmitter());
            transmitterThread.setName("Transmitter");
            transmitterThread.start();
            receiverThread = new Thread(new Receiver());
            receiverThread.setName("Receiver");
            receiverThread.start();

            try {
                Thread.sleep(100);
            }
            catch (InterruptedException ex) {
                LOGGER.log(Level.SEVERE, null, ex);
            }

            connected = true;
            ClearSync();
            TransmitPing();
            WaitSync();

            try {
                Thread.sleep(100);
            }
            catch (InterruptedException ex) {
                LOGGER.log(Level.SEVERE, null, ex);
            }

            LOGGER.log(Level.WARNING, "Connected\n");

            try {
                Thread.sleep(100);
            }
            catch (InterruptedException ex) {
                LOGGER.log(Level.SEVERE, null, ex);
            }
            QCmdProcessor qcmdp = MainFrame.mainframe.getQcmdprocessor();
            qcmdp.AppendToQueue(new QCmdTransmitGetFWVersion());

            try {
                Thread.sleep(100);
            }
            catch (InterruptedException ex) {
                LOGGER.log(Level.SEVERE, null, ex);
            }
            qcmdp.WaitQueueFinished();

            QCmdMemRead1Word q1 = new QCmdMemRead1Word(targetProfile.getCPUIDCodeAddr());
            qcmdp.AppendToQueue(q1);
            targetProfile.setCPUIDCode(q1.getResult());

            QCmdMemRead q;

            q = new QCmdMemRead(targetProfile.getCPUSerialAddr(), targetProfile.getCPUSerialLength());
            qcmdp.AppendToQueue(q);
            targetProfile.setCPUSerial(q.getResult());

            // q = new QCmdMemRead(targetProfile.getOTPAddr(), 32);
            // qcmdp.AppendToQueue(q);
            // ByteBuffer otpInfo = q.getResult();

            // q = new QCmdMemRead(targetProfile.getOTPAddr() + 32, 256);
            // qcmdp.AppendToQueue(q);

            // ByteBuffer signature = q.getResult();

            // boolean signaturevalid = false;
            // if (signature == null) {
            //     LOGGER.log(Level.INFO, "Cannot obtain signature, upgrade firmware?");
            // }

            // boolean signing = false;

            // if (signing && !signaturevalid) {

            //     qcmdp.WaitQueueFinished();
            //     ByteBuffer writeotpinfo = targetProfile.CreateOTPInfo();
            //     byte[] sign = HWSignature.Sign(targetProfile.getCPUSerial(), writeotpinfo);
            //     qcmdp.AppendToQueue(new QCmdWriteMem(targetProfile.getBKPSRAMAddr(), bb2ba(writeotpinfo)));
            //     qcmdp.AppendToQueue(new QCmdWriteMem(targetProfile.getBKPSRAMAddr() + 32, sign));

            //     CRC32 zcrc = new CRC32();
            //     writeotpinfo.rewind();
            //     zcrc.update(bb2ba(writeotpinfo));
            //     zcrc.update(sign);
            //     int zcrcv = (int) zcrc.getValue();
            //     System.out.println(String.format("Key crc: %08X", zcrcv));

            //     byte crc[] = new byte[4];
            //     crc[0] = (byte) (zcrcv & 0xFF);
            //     crc[1] = (byte) ((zcrcv >> 8) & 0xFF);
            //     crc[2] = (byte) ((zcrcv >> 16) & 0xFF);
            //     crc[3] = (byte) ((zcrcv >> 24) & 0xFF);
            //     qcmdp.AppendToQueue(new QCmdWriteMem(targetProfile.getBKPSRAMAddr() + 32 + 256, crc));

            //     /* Validate from bkpsram */
            //     qcmdp.WaitQueueFinished();
            //     q = new QCmdMemRead(targetProfile.getBKPSRAMAddr(), 32);
            //     qcmdp.AppendToQueue(q);
            //     ByteBuffer otpInfo2 = q.getResult();

            //     q = new QCmdMemRead(targetProfile.getBKPSRAMAddr() + 32, 256);
            //     qcmdp.AppendToQueue(q);
            //     ByteBuffer signature2 = q.getResult();

            //     boolean signaturevalid2 = HWSignature.Verify(targetProfile.getCPUSerial(), otpInfo2, bb2ba(signature2));
            //     if (signaturevalid2) {
            //         System.out.println("BPKSRAM signature valid");
            //     }
            //     else {
            //         System.out.println("BPKSRAM signature invalid");
            //         return false;
            //     }

            //     System.out.println("<otpinfo>");
            //     HWSignature.printByteArray(bb2ba(otpInfo2));
            //     System.out.println("</otpinfo>");

            //     System.out.println("<signature>");
            //     HWSignature.printByteArray(sign);
            //     System.out.println("</signature>");

            // }

            ShowConnect();

            return true;

        }
        catch (Exception ex) {
            LOGGER.log(Level.SEVERE, null, ex);
            ShowDisconnect();
            return false;
        }
    }

    static final byte OUT_ENDPOINT = 0x02;
    static final byte IN_ENDPOINT = (byte) 0x82;
    static final int TIMEOUT = 1000;

    @Override
    public int writeBytes(byte[] data) {

        ByteBuffer buffer = ByteBuffer.allocateDirect(data.length);
        buffer.put(data);

        IntBuffer transfered = IntBuffer.allocate(1);

        int result = LibUsb.bulkTransfer(handle, (byte) OUT_ENDPOINT, buffer, transfered, TIMEOUT);
        if (result != LibUsb.SUCCESS) { /* handle error -99 below */

            if (result == -99) {
                /*
                * Filter out error -99 ... seems to pop up every now and then but does not lead to connection loss  
                * this "bug" will likely  be resolved after libusb update
                */
                // LOGGER.log(Level.INFO, "USB connection not happy: " + result);
                return result;
            }

            String errstr;
            switch (result) {
                case -1:  errstr = "Input/output error"; break;
                case -2:  errstr = "Invalid parameter"; break;
                case -3:  errstr = "Access denied (insufficient permissions?)"; break;
                case -4:  errstr = "Device may have been disconnected"; break;
                case -5:  errstr = "Device not found"; break;
                case -6:  errstr = "Device busy"; break;
                case -7:  errstr = "Operation timed out"; break;
                case -8:  errstr = "Overflow"; break;
                case -9:  errstr = "Pipe error"; break;
                case -10: errstr = "System call interrupted"; break;
                case -11: errstr = "Insufficient memory"; break;
                case -12: errstr = "Operation not supported or unimplemented"; break;
                default:  errstr = Integer.toString(result); break;
            }
            LOGGER.log(Level.SEVERE, "USB connection failed: " + errstr);
            QCmdProcessor.getQCmdProcessor().Abort();
        }
        return result;
    }

    @Override
    public void TransmitRecallPreset(int presetNo) {
        byte[] data = new byte[5];
        data[0] = 'A';
        data[1] = 'x';
        data[2] = 'o';
        data[3] = 'T';
        data[4] = (byte) presetNo;
        writeBytes(data);
    }

    @Override
    public void BringToDFU() {
        byte[] data = new byte[4];
        data[0] = 'A';
        data[1] = 'x';
        data[2] = 'o';
        data[3] = 'D';
        writeBytes(data);
    }

    @Override
    public void TransmitGetFWVersion() {
        byte[] data = new byte[4];
        data[0] = 'A';
        data[1] = 'x';
        data[2] = 'o';
        data[3] = 'V';
        writeBytes(data);
    }

    @Override
    public void TransmitGetSpilinkSynced() {
        byte[] data = new byte[4];
        data[0] = 'A';
        data[1] = 'x';
        data[2] = 'o';
        data[3] = 'Y';
        writeBytes(data);
    }

    @Override
    public void SendMidi(int m0, int m1, int m2) {
        if (isConnected()) {
            byte[] data = new byte[7];
            data[0] = 'A';
            data[1] = 'x';
            data[2] = 'o';
            data[3] = 'M';
            data[4] = (byte) m0;
            data[5] = (byte) m1;
            data[6] = (byte) m2;
            writeBytes(data);
        }
    }

    @Override
    public void SendUpdatedPreset(byte[] b) {
        byte[] data = new byte[8];
        data[0] = 'A';
        data[1] = 'x';
        data[2] = 'o';
        data[3] = 'R';
        int len = b.length;
        data[4] = (byte) len;
        data[5] = (byte) (len >> 8);
        data[6] = (byte) (len >> 16);
        data[7] = (byte) (len >> 24);
        writeBytes(data);
        writeBytes(b);
    }

    @Override
    public void SelectPort() {
        USBPortSelectionDlg spsDlg = new USBPortSelectionDlg(null, true, cpuid);
        spsDlg.setVisible(true);
        cpuid = spsDlg.getCPUID();
        String name = prefs.getBoardName(cpuid);
        if (cpuid == null) return;
        if (name == null) {
            LOGGER.log(Level.INFO, "Selecting CPUID: {0} for connection.", cpuid);
        }
        else {
            LOGGER.log(Level.INFO, "Selecting \"{0}\" for connection.", new Object[]{name});
        }
    }

    static private USBBulkConnection conn = null;

    public static Connection GetConnection() {
        if (conn == null) {
            conn = new USBBulkConnection();
        }
        return conn;
    }

    @Override
    public void TransmitGetFileInfo(String filename) {
        byte[] data = new byte[15 + filename.length()];
        data[0] = 'A';
        data[1] = 'x';
        data[2] = 'o';
        data[3] = 'C';
        data[4] = 0;
        data[5] = 0;
        data[6] = 0;
        data[7] = 0;
        data[8] = 0;
        data[9] = 'I';
        data[10] = 0;
        data[11] = 0;
        data[12] = 0;
        data[13] = 0;
        int i = 14;
        for (int j = 0; j < filename.length(); j++) {
            data[i++] = (byte) filename.charAt(j);
        }
        data[i] = 0;
        ClearSync();
        writeBytes(data);
        WaitSync();
    }

    class Sync {

        boolean Acked = false;
    }
    final Sync sync;
    final Sync readsync;

    @Override
    public void ClearSync() {
        synchronized (sync) {
            sync.Acked = false;
        }
    }

    @Override
    public boolean WaitSync(int msec) {
        synchronized (sync) {
                if (sync.Acked) {
                    return sync.Acked;
                }
                try {
                    sync.wait(msec);
                }
                catch (InterruptedException ex) {
                    // LOGGER.log(Level.SEVERE, "Sync wait interrupted");
                }
            return sync.Acked;
        }
    }

    @Override
    public boolean WaitSync() {
        return WaitSync(1000);
    }

    @Override
    public void ClearReadSync() {
        synchronized (readsync) {
            readsync.Acked = false;
        }
    }

    @Override
    public boolean WaitReadSync() {
        synchronized (readsync) {
            if (readsync.Acked) {
                return readsync.Acked;
            }
            try {
                readsync.wait(1000);
            }
            catch (InterruptedException ex) {
                // LOGGER.log(Level.SEVERE, "Sync wait interrupted");
            }
            return readsync.Acked;
        }
    }

    private final byte[] startPckt = new byte[]{(byte) ('A'), (byte) ('x'), (byte) ('o'), (byte) ('s')};
    private final byte[] stopPckt = new byte[]{(byte) ('A'), (byte) ('x'), (byte) ('o'), (byte) ('S')};
    private final byte[] pingPckt = new byte[]{(byte) ('A'), (byte) ('x'), (byte) ('o'), (byte) ('p')};
    private final byte[] getFileListPckt = new byte[]{(byte) ('A'), (byte) ('x'), (byte) ('o'), (byte) ('d')};
    private final byte[] copyToFlashPckt = new byte[]{(byte) ('A'), (byte) ('x'), (byte) ('o'), (byte) ('F')};

    @Override
    public void TransmitStart() {
        writeBytes(startPckt);
    }

    @Override
    public void TransmitStop() {
        writeBytes(stopPckt);
    }

    @Override
    public void TransmitGetFileList() {
        writeBytes(getFileListPckt);
    }

    @Override
    public void TransmitPing() {
        writeBytes(pingPckt);
    }

    @Override
    public void TransmitCopyToFlash() {
        writeBytes(copyToFlashPckt);
    }

    @Override
    public int UploadFragment(byte[] buffer, int offset) {
        byte[] data = new byte[12];
        data[0] = 'A';
        data[1] = 'x';
        data[2] = 'o';
        data[3] = 'W';
        int tvalue = offset;
        int nRead = buffer.length;
        data[4] = (byte) tvalue;
        data[5] = (byte) (tvalue >> 8);
        data[6] = (byte) (tvalue >> 16);
        data[7] = (byte) (tvalue >> 24);
        data[8] = (byte) (nRead);
        data[9] = (byte) (nRead >> 8);
        data[10] = (byte) (nRead >> 16);
        data[11] = (byte) (nRead >> 24);
        ClearSync();
        int result = writeBytes(data);
        result |= writeBytes(buffer);
        if (!WaitSync()) {
            result |= 1;
        };
        LOGGER.log(Level.INFO, "Block uploaded @ 0x{0} length {1}",
                   new Object[]{Integer.toHexString(offset).toUpperCase(),
                   Integer.toString(buffer.length)});
        return result;
    }

    @Override
    public void TransmitVirtualButton(int b_or, int b_and, int enc1, int enc2, int enc3, int enc4) {
        byte[] data = new byte[16];
        data[0] = 'A';
        data[1] = 'x';
        data[2] = 'o';
        data[3] = 'B';
        data[4] = (byte) b_or;
        data[5] = (byte) (b_or >> 8);
        data[6] = (byte) (b_or >> 16);
        data[7] = (byte) (b_or >> 24);
        data[8] = (byte) b_and;
        data[9] = (byte) (b_and >> 8);
        data[10] = (byte) (b_and >> 16);
        data[11] = (byte) (b_and >> 24);
        data[12] = (byte) (enc1);
        data[13] = (byte) (enc2);
        data[14] = (byte) (enc3);
        data[15] = (byte) (enc4);
        writeBytes(data);
    }

    @Override
    public void TransmitCreateFile(String filename, int size) {
        byte[] data = new byte[9 + filename.length()];
        data[0] = 'A';
        data[1] = 'x';
        data[2] = 'o';
        data[3] = 'C';
        data[4] = (byte) size;
        data[5] = (byte) (size >> 8);
        data[6] = (byte) (size >> 16);
        data[7] = (byte) (size >> 24);
        int i = 8;
        for (int j = 0; j < filename.length(); j++) {
            data[i++] = (byte) filename.charAt(j);
        }
        data[i] = 0;
        ClearSync();
        writeBytes(data);
        WaitSync();
    }

    @Override
    public void TransmitCreateFile(String filename, int size, Calendar date) {
        byte[] data = new byte[15 + filename.length()];
        data[0] = 'A';
        data[1] = 'x';
        data[2] = 'o';
        data[3] = 'C';
        data[4] = (byte) size;
        data[5] = (byte) (size >> 8);
        data[6] = (byte) (size >> 16);
        data[7] = (byte) (size >> 24);
        data[8] = 0;
        data[9] = 'f';
        int dy = date.get(Calendar.YEAR);
        int dm = date.get(Calendar.MONTH) + 1;
        int dd = date.get(Calendar.DAY_OF_MONTH);
        int th = date.get(Calendar.HOUR_OF_DAY);
        int tm = date.get(Calendar.MINUTE);
        int ts = date.get(Calendar.SECOND);
        int t = ((dy - 1980) * 512) | (dm * 32) | dd;
        int d = (th * 2048) | (tm * 32) | (ts / 2);
        data[10] = (byte) (t & 0xff);
        data[11] = (byte) (t >> 8);
        data[12] = (byte) (d & 0xff);
        data[13] = (byte) (d >> 8);
        int i = 14;
        for (int j = 0; j < filename.length(); j++) {
            data[i++] = (byte) filename.charAt(j);
        }
        data[i] = 0;
        ClearSync();
        writeBytes(data);
        WaitSync();
    }

    @Override
    public void TransmitDeleteFile(String filename) {
        byte[] data = new byte[15 + filename.length()];
        data[0] = 'A';
        data[1] = 'x';
        data[2] = 'o';
        data[3] = 'C';
        data[4] = 0;
        data[5] = 0;
        data[6] = 0;
        data[7] = 0;
        data[8] = 0;
        data[9] = 'D';
        data[10] = 0;
        data[11] = 0;
        data[12] = 0;
        data[13] = 0;
        int i = 14;
        for (int j = 0; j < filename.length(); j++) {
            data[i++] = (byte) filename.charAt(j);
        }
        data[i] = 0;
        ClearSync();
        writeBytes(data);
        WaitSync();
    }

    @Override
    public void TransmitChangeWorkingDirectory(String path) {
        byte[] data = new byte[15 + path.length()];
        data[0] = 'A';
        data[1] = 'x';
        data[2] = 'o';
        data[3] = 'C';
        data[4] = 0;
        data[5] = 0;
        data[6] = 0;
        data[7] = 0;
        data[8] = 0;
        data[9] = 'C';
        data[10] = 0;
        data[11] = 0;
        data[12] = 0;
        data[13] = 0;
        int i = 14;
        for (int j = 0; j < path.length(); j++) {
            data[i++] = (byte) path.charAt(j);
        }
        data[i] = 0;
        ClearSync();
        writeBytes(data);
        WaitSync();
    }

    @Override
    public void TransmitCreateDirectory(String filename, Calendar date) {
        byte[] data = new byte[15 + filename.length()];
        data[0] = 'A';
        data[1] = 'x';
        data[2] = 'o';
        data[3] = 'C';
        data[4] = 0;
        data[5] = 0;
        data[6] = 0;
        data[7] = 0;
        data[8] = 0;
        data[9] = 'd';
        data[10] = 0;
        data[11] = 0;
        data[12] = 0;
        data[13] = 0;

        int i = 14;
        for (int j = 0; j < filename.length(); j++) {
            data[i++] = (byte) filename.charAt(j);
        }
        data[i] = 0;
        ClearSync();
        writeBytes(data);
        WaitSync();
    }

    @Override
    public void TransmitAppendFile(byte[] buffer) {
        byte[] data = new byte[8];
        data[0] = 'A';
        data[1] = 'x';
        data[2] = 'o';
        data[3] = 'A';
        int size = buffer.length;
        // LOGGER.log(Level.INFO, "Append size: " + buffer.length);
        data[4] = (byte) size;
        data[5] = (byte) (size >> 8);
        data[6] = (byte) (size >> 16);
        data[7] = (byte) (size >> 24);
        ClearSync();
        writeBytes(data);
        writeBytes(buffer);
        WaitSync();
    }

    @Override
    public void TransmitCloseFile() {
        byte[] data = new byte[4];
        data[0] = 'A';
        data[1] = 'x';
        data[2] = 'o';
        data[3] = 'c';
        ClearSync();
        writeBytes(data);
        WaitSync();
    }

    @Override
    public void TransmitMemoryRead(int addr, int length) {
        byte[] data = new byte[12];
        data[0] = 'A';
        data[1] = 'x';
        data[2] = 'o';
        data[3] = 'r';
        data[4] = (byte) addr;
        data[5] = (byte) (addr >> 8);
        data[6] = (byte) (addr >> 16);
        data[7] = (byte) (addr >> 24);
        data[8] = (byte) length;
        data[9] = (byte) (length >> 8);
        data[10] = (byte) (length >> 16);
        data[11] = (byte) (length >> 24);
        ClearSync();
        writeBytes(data);
        WaitSync();
    }

    @Override
    public void TransmitMemoryRead1Word(int addr) {
        byte[] data = new byte[8];
        data[0] = 'A';
        data[1] = 'x';
        data[2] = 'o';
        data[3] = 'y';
        data[4] = (byte) addr;
        data[5] = (byte) (addr >> 8);
        data[6] = (byte) (addr >> 16);
        data[7] = (byte) (addr >> 24);
        ClearSync();
        writeBytes(data);
        WaitSync();
    }

    @Override
    public void TransmitCosts() {
        short uUIMidiCost = prefs.getUiMidiThreadCost();
        byte  uDspLimit200 = (byte)(prefs.getDspLimitPercent()*2);

        byte[] data = new byte[7];
        data[0] = 'A';
        data[1] = 'x';
        data[2] = 'o';
        data[3] = 'U';
        data[4] = (byte) uUIMidiCost;
        data[5] = (byte) (uUIMidiCost >> 8);
        data[6] = uDspLimit200;
        ClearSync();
        writeBytes(data);
        WaitSync();
    }


    class Receiver implements Runnable {

        @Override
        public void run() {

            ByteBuffer recvbuffer = ByteBuffer.allocateDirect(4096);
            IntBuffer transfered = IntBuffer.allocate(1);

            while (!disconnectRequested) {
                int result = LibUsb.bulkTransfer(handle, (byte) IN_ENDPOINT, recvbuffer, transfered, TIMEOUT);

                if (result != LibUsb.SUCCESS) {
                    // LOGGER.log(Level.INFO, "Receive: " + result);
                }
                else {
                    int sz = transfered.get(0);
                    if (sz != 0) {
                        // LOGGER.log(Level.INFO, "Receive sz: " + sz);
                    }
                    for (int i = 0; i < sz; i++) {
                        processByte(recvbuffer.get(i));
                    }
                }
            }

            // LOGGER.log(Level.INFO, "Receiver: thread stopped");

            MainFrame.mainframe.qcmdprocessor.Abort();
            MainFrame.mainframe.qcmdprocessor.AppendToQueue(new QCmdShowDisconnect());
        }
    }

    class Transmitter implements Runnable {

        @Override
        public void run() {
            while (!disconnectRequested) {
                try {
                    QCmdSerialTask cmd = queueSerialTask.take();
                    QCmd response = cmd.Do(USBBulkConnection.this);
                    if (response != null) {
                        QCmdProcessor.getQCmdProcessor().getQueueResponse().add(response);
                    }
                }
                catch (InterruptedException ex) {
                    LOGGER.log(Level.SEVERE, null, ex);
                }
            }
            // LOGGER.log(Level.INFO, "Transmitter: thread stopped");
            MainFrame.mainframe.qcmdprocessor.Abort();
            MainFrame.mainframe.qcmdprocessor.AppendToQueue(new QCmdShowDisconnect());
        }
    }

    private Boolean isSDCardPresent = null;
    
    public void SetSDCardPresent(boolean i) {
        if ((isSDCardPresent != null) && (i == isSDCardPresent)) return;
        isSDCardPresent = i;
        if (isSDCardPresent) {
            ShowSDCardMounted();
        }
        else {
            ShowSDCardUnmounted();            
        }
    }

    @Override
    public boolean GetSDCardPresent() {
        if (isSDCardPresent == null) return false;
        return isSDCardPresent;
    }

    private int connectionFlags = 0;

    public void SetConnectionFlags(int newConnectionFlags) {
        if(newConnectionFlags != connectionFlags) {
            connectionFlags = newConnectionFlags;
            ShowConnectionFlags(connectionFlags);
        }
    }

    @Override
    public int GetConnectionFlags() {
        return connectionFlags;
    }

    int CpuId0 = 0;
    int CpuId1 = 0;
    int CpuId2 = 0;
    int fwcrc = -1;

    void Acknowledge(final int ConnectionFlags, final int DSPLoad, final int PatchID, final int Voltages, final int patchIndex, final int sdcardPresent) {
        synchronized (sync) {
            sync.Acked = true;
            sync.notifyAll();
        }
        SwingUtilities.invokeLater(new Runnable() {
            @Override
            public void run() {
                if (patch != null) {
                    if ((patch.GetIID() != PatchID) && patch.IsLocked()) {
                        patch.Unlock();
                    }
                    else {
                        boolean dspOverload = 0 != (ConnectionFlags & 1);
                        patch.UpdateDSPLoad(DSPLoad, dspOverload);
                    }
                }
                MainFrame.mainframe.showPatchIndex(patchIndex);
                targetProfile.setVoltages(Voltages);
                SetSDCardPresent(sdcardPresent!=0);
                SetConnectionFlags(ConnectionFlags);
            }
        });
    }

    void RPacketParamChange(final int index, final int value, final int patchID) {
        SwingUtilities.invokeLater(new Runnable() {
            @Override
            public
                    void run() {
                if (patch == null) {
                    // LOGGER.log(Level.INFO, "Rx paramchange patch null {0} {1}", new Object[]{index, value});
                    return;
                }
                if (!patch.IsLocked()) {
                    return;

                }
                if (patch.GetIID() != patchID) {
                    patch.Unlock();
                    return;
                }
                if (index >= patch.ParameterInstances.size()) {
                    LOGGER.log(Level.INFO, "Rx paramchange index out of range{0} {1}",
                               new Object[]{index, value});

                    return;
                }
                ParameterInstance pi = patch.ParameterInstances.get(index);

                if (pi == null) {
                    LOGGER.log(Level.INFO, "Rx paramchange parameterInstance null{0} {1}",
                               new Object[]{index, value});
                    return;
                }

                if (!pi.GetNeedsTransmit()) {
                    pi.SetValueRaw(value);
                }
                // System.out.println("rcv ppc objname:" + pi.axoObj.getInstanceName() + " pname:"+ pi.name);
            }
        });

    }

    enum ReceiverState {

        header,
        ackPckt,            /* general acknowledge */
        paramchangePckt,    /* parameter changed */
        lcdPckt,            /* lcd screen bitmap readback */
        displayPcktHdr,     /* object display readbac */
        displayPckt,        /* object display readback */
        textPckt,           /* text message to display in log */
        sdinfo,             /* sdcard info */
        fileinfo,           /* file listing entry */
        memread,            /* one-time programmable bytes */
        memread1word,       /* one-time programmable bytes */
        fwversion
    };

    /*
     * Protocol documentation:
     * "AxoP" + bb + vvvv -> parameter change index bb (16bit), value vvvv (32bit)
     */
    private ReceiverState state = ReceiverState.header;
    private int headerstate;
    private int[] packetData = new int[64];
    private int dataIndex = 0;  /* in bytes */
    private int dataLength = 0; /* in bytes */
    private CharBuffer textRcvBuffer = CharBuffer.allocate(256);
    private ByteBuffer lcdRcvBuffer = ByteBuffer.allocate(256);
    private ByteBuffer sdinfoRcvBuffer = ByteBuffer.allocate(12);
    private ByteBuffer fileinfoRcvBuffer = ByteBuffer.allocate(256);
    private ByteBuffer memReadBuffer = ByteBuffer.allocate(16 * 4);
    // private int memReadAddr;
    private int memReadLength;
    private int memReadValue;
    private byte[] fwversion = new byte[4];
    private int patchentrypoint;

    @Override
    public ByteBuffer getMemReadBuffer() {
        return memReadBuffer;
    }

    @Override
    public int getMemRead1Word() {
        return memReadValue;
    }

    void storeDataByte(int c) {
        switch (dataIndex & 0x3) {
            case 0:
                packetData[dataIndex >> 2] = c;
                break;
            case 1:
                packetData[dataIndex >> 2] += (c << 8);
                break;
            case 2:
                packetData[dataIndex >> 2] += (c << 16);
                break;
            case 3:
                packetData[dataIndex >> 2] += (c << 24);
                break;
        }
        // System.out.println("s " + dataIndex + "  v=" + Integer.toHexString(packetData[dataIndex>>2]) + " c=");
        dataIndex++;

    }

    void DisplayPackHeader(int i1, int i2) {
        if (i2 > 1024) {
            LOGGER.log(Level.FINE, "Lots of data coming! {0} / {1}",
                       new Object[]{Integer.toHexString(i1),
                       Integer.toHexString(i2)});
        }
        else {
            // LOGGER.log(Level.INFO, "OK! " + Integer.toHexString(i1) + " / " + Integer.toHexString(i2));
        }

        if (i2 > 0) {
            dataLength = i2 * 4;
            dataIndex = 0;
            dispData = ByteBuffer.allocate(dataLength);
            dispData.order(ByteOrder.LITTLE_ENDIAN);
            state = ReceiverState.displayPckt;
        }
        else {
            GoIdleState();
        }
    }

    void DistributeToDisplays(final ByteBuffer dispData) {
        // LOGGER.log(Level.INFO, "Distr1");
        try {
            if (patch == null) {
                return;
            }
            if (!patch.IsLocked()) {
                return;
            }
            if (patch.DisplayInstances == null) {
                return;
            }
            // LOGGER.log(Level.INFO, "Distr2");
            SwingUtilities.invokeAndWait(new Runnable() {
                @Override
                public void run() {
                    dispData.rewind();
                    for (DisplayInstance d : patch.DisplayInstances) {
                        d.ProcessByteBuffer(dispData);
                    }
                }
            });
        }
        catch (InterruptedException ex) {
            LOGGER.log(Level.SEVERE, null, ex);
        }
        catch (InvocationTargetException ex) {
            LOGGER.log(Level.SEVERE, null, ex);
        }
    }

    void GoIdleState() {
        headerstate = 0;
        state = ReceiverState.header;
    }
    ByteBuffer dispData;

    int LCDPacketRow = 0;

    void processByte(byte cc) {
        // LOGGER.log(Level.SEVERE,"AxoP c="+(char)c+"="+Integer.toHexString(c)+" s="+Integer.toHexString(state));
        int c = cc & 0xff;
        // System.out.println("AxoP c="+(char)c+"="+Integer.toHexString(c));
        switch (state) {
            case header:
                switch (headerstate) {
                    case 0:
                        if (c == 'A') {
                            headerstate = 1;
                        }
                        break;
                    case 1:
                        if (c == 'x') {
                            headerstate = 2;
                        }
                        else {
                            GoIdleState();
                        }
                        break;
                    case 2:
                        if (c == 'o') {
                            headerstate = 3;
                        }
                        else {
                            GoIdleState();
                        }
                        break;
                    case 3:
                        switch (c) {
                            case 'Q':
                                state = ReceiverState.paramchangePckt;
                                // System.out.println("param packet start");
                                dataIndex = 0;
                                dataLength = 12;
                                break;
                            case 'A':
                                state = ReceiverState.ackPckt;
                                // System.out.println("ack packet start");
                                dataIndex = 0;
                                dataLength = 24;
                                break;
                            case 'D':
                                state = ReceiverState.displayPcktHdr;
                                // System.out.println("display packet start");
                                dataIndex = 0;
                                dataLength = 8;
                                break;
                            case 'T':
                                state = ReceiverState.textPckt;
                                // System.out.println("text packet start");
                                textRcvBuffer.clear();
                                dataIndex = 0;
                                dataLength = 255;
                                break;
                            case '0':
                            case '1':
                            case '2':
                            case '3':
                            case '4':
                            case '5':
                            case '6':
                            case '7':
                            case '8':
                                LCDPacketRow = c - '0';
                                state = ReceiverState.lcdPckt;
                                // System.out.println("text packet start");
                                lcdRcvBuffer.rewind();
                                dataIndex = 0;
                                dataLength = 128;
                                break;
                            case 'd':
                                state = ReceiverState.sdinfo;
                                sdinfoRcvBuffer.rewind();
                                dataIndex = 0;
                                dataLength = 12;
                                break;
                            case 'f':
                                state = ReceiverState.fileinfo;
                                fileinfoRcvBuffer.clear();
                                dataIndex = 0;
                                dataLength = 8;
                                break;
                            case 'r':
                                state = ReceiverState.memread;
                                memReadBuffer.clear();
                                dataIndex = 0;
                                break;
                            case 'y':
                                state = ReceiverState.memread1word;
                                dataIndex = 0;
                                break;
                            case 'V':
                                state = ReceiverState.fwversion;
                                dataIndex = 0;
                                break;
                            default:
                                GoIdleState();
                                break;
                        }
                        break;
                    default:
                        LOGGER.log(Level.SEVERE, "Receiver: invalid header");
                        GoIdleState();

                        break;
                }
                break;
            case paramchangePckt:
                if (dataIndex < dataLength) {
                    storeDataByte(c);
                }
                // System.out.println("pch packet i=" +dataIndex + " v=" + c + " c="+ (char)(cc));
                if (dataIndex == dataLength) {
                    // System.out.println("param packet complete 0x" + Integer.toHexString(packetData[1]) + "    0x" + Integer.toHexString(packetData[0]));
                    RPacketParamChange(packetData[2], packetData[1], packetData[0]);
                    GoIdleState();
                }
                break;
            case ackPckt:
                if (dataIndex < dataLength) {
                    // System.out.println("ack packet i=" +dataIndex + " v=" + c + " c="+ (char)(cc));
                    storeDataByte(c);
                }
                if (dataIndex == dataLength) {
                    // System.out.println("ack packet complete");
                    Acknowledge(packetData[0], packetData[1], packetData[2], packetData[3], packetData[4], packetData[5]);
                    GoIdleState();
                }
                break;
            case lcdPckt:
                if (dataIndex < dataLength) {
                    // System.out.println("lcd packet i=" +dataIndex + " v=" + c + " c="+ (char)(cc));
                    lcdRcvBuffer.put(cc);
                    dataIndex++;
                }
                if (dataIndex == dataLength) {
                    lcdRcvBuffer.rewind();
                    // MainFrame.mainframe.remote.updateRow(LCDPacketRow, lcdRcvBuffer);
                    GoIdleState();
                }
                break;
            case displayPcktHdr:
                if (dataIndex < dataLength) {
                    storeDataByte(c);
                }
                // System.out.println("pch packet i=" +dataIndex + " v=" + c + " c="+ (char)(cc));
                if (dataIndex == dataLength) {
                    DisplayPackHeader(packetData[0], packetData[1]);
                }
                break;
            case displayPckt:
                if (dataIndex < dataLength) {
                    dispData.put(cc);
                    dataIndex++;
                }
                if (dataIndex == dataLength) {
                    DistributeToDisplays(dispData);
                    GoIdleState();
                }
                break;
            case textPckt:
                if (c != 0) {
                    textRcvBuffer.append((char) cc);
                }
                else {
                    // textRcvBuffer.append((char) cc);
                    textRcvBuffer.limit(textRcvBuffer.position());
                    textRcvBuffer.rewind();
                    if (textRcvBuffer.toString().toLowerCase().contains("file error: fr_no_file, filename:\"/start.bin\"")) {
                        /* Filter out error if SD card is connected but no start.bin is found */
                        LOGGER.log(Level.INFO, "SD card connected, no startup patch found.");
                    }
                    else {
                        LOGGER.log(Level.WARNING, "{0}", textRcvBuffer.toString());
                    }
                    GoIdleState();
                }
                break;
            case sdinfo:
                if (dataIndex < dataLength) {
                    sdinfoRcvBuffer.put(cc);
                    dataIndex++;
                }
                if (dataIndex == dataLength) {
                    sdinfoRcvBuffer.rewind();
                    sdinfoRcvBuffer.order(ByteOrder.LITTLE_ENDIAN);
                    // LOGGER.log(Level.INFO, "sdinfo: "
                    // + sdinfoRcvBuffer.asIntBuffer().get(0) + " "
                    // + sdinfoRcvBuffer.asIntBuffer().get(1) + " "
                    // + sdinfoRcvBuffer.asIntBuffer().get(2));
                    SDCardInfo.getInstance().SetInfo(sdinfoRcvBuffer.asIntBuffer().get(0), sdinfoRcvBuffer.asIntBuffer().get(1), sdinfoRcvBuffer.asIntBuffer().get(2));
                    GoIdleState();
                }
                break;
            case fileinfo:
                if ((dataIndex < dataLength) || (c != 0)) {
                    fileinfoRcvBuffer.put(cc);
                    // System.out.println("fileinfo \'" + (char) c + "\' = " + c);
                    dataIndex++;
                }
                else {
                    fileinfoRcvBuffer.put((byte) c);
                    fileinfoRcvBuffer.order(ByteOrder.LITTLE_ENDIAN);
                    fileinfoRcvBuffer.limit(fileinfoRcvBuffer.position());
                    fileinfoRcvBuffer.rewind();
                    int size = fileinfoRcvBuffer.getInt();
                    int timestamp = fileinfoRcvBuffer.getInt();
                    CharBuffer cb = Charset.forName("ISO-8859-1").decode(fileinfoRcvBuffer);
                    String fname = cb.toString();
                    // strip trailing null
                    if (fname.charAt(fname.length() - 1) == (char) 0) {
                        fname = fname.substring(0, fname.length() - 1);
                    }
                    SDCardInfo.getInstance().AddFile(fname, size, timestamp);
                    // LOGGER.log(Level.INFO, "fileinfo: " + cb.toString());                    
                    GoIdleState();
                    if (fname.equals("/")) {
                        /* end of index */
                        // System.out.println("sdfilelist done");
                        synchronized (readsync) {
                            readsync.Acked = true;
                            readsync.notifyAll();
                        }
                    }
                }
                break;
            case memread:
                switch (dataIndex) {
                    // case 0:
                    //     memReadAddr = (cc & 0xFF);
                    //     break;
                    // case 1:
                    //     memReadAddr += (cc & 0xFF) << 8;
                    //     break;
                    // case 2:
                    //     memReadAddr += (cc & 0xFF) << 16;
                    //     break;
                    // case 3:
                    //     memReadAddr += (cc & 0xFF) << 24;
                    //     break;
                    case 4:
                        memReadLength = (cc & 0xFF);
                        break;
                    case 5:
                        memReadLength += (cc & 0xFF) << 8;
                        break;
                    case 6:
                        memReadLength += (cc & 0xFF) << 16;
                        break;
                    case 7:
                        memReadLength += (cc & 0xFF) << 24;
                        break;
                    case 8:
                        memReadBuffer = ByteBuffer.allocate(memReadLength);
                        memReadBuffer.rewind();
                    default:
                        memReadBuffer.put(cc);
                        if (dataIndex == memReadLength + 7) {
                            memReadBuffer.rewind();
                            memReadBuffer.order(ByteOrder.LITTLE_ENDIAN);
                            // System.out.println("memread offset 0x" + Integer.toHexString(memReadAddr));
                            // int i = 0;
                            // while (memReadBuffer.hasRemaining()) {
                            //     System.out.print(" " + String.format("%02X", memReadBuffer.get()));
                            //     i++;
                            //     // if ((i % 4) == 0) {
                            //         // System.out.print(" ");
                            //     // }
                            //     if ((i % 32) == 0) {
                            //         System.out.println();
                            //     }
                            // }
                            // System.out.println();
                            synchronized (readsync) {
                                readsync.Acked = true;
                                readsync.notifyAll();
                            }
                            GoIdleState();
                        }
                }
                dataIndex++;
                break;

            case memread1word:
                switch (dataIndex) {
                    // case 0:
                    //     memReadAddr = (cc & 0xFF);
                    //     break;
                    // case 1:
                    //     memReadAddr += (cc & 0xFF) << 8;
                    //     break;
                    // case 2:
                    //     memReadAddr += (cc & 0xFF) << 16;
                    //     break;
                    // case 3:
                    //     memReadAddr += (cc & 0xFF) << 24;
                    //     break;
                    case 4:
                        memReadValue = (cc & 0xFF);
                        break;
                    case 5:
                        memReadValue += (cc & 0xFF) << 8;
                        break;
                    case 6:
                        memReadValue += (cc & 0xFF) << 16;
                        break;
                    case 7:
                        memReadValue += (cc & 0xFF) << 24;
                        // System.out.println(String.format("addr %08X value %08X", memReadAddr, memReadValue));
                        synchronized (readsync) {
                            readsync.Acked = true;
                            readsync.notifyAll();
                        }
                        GoIdleState();
                }
                dataIndex++;
                break;

            case fwversion:
                switch (dataIndex) {
                    case 0:
                        fwversion[0] = cc;
                        break;
                    case 1:
                        fwversion[1] = cc;
                        break;
                    case 2:
                        fwversion[2] = cc;
                        break;
                    case 3:
                        fwversion[3] = cc;
                        break;
                    case 4:
                        fwcrc = (cc & 0xFF) << 24;
                        break;
                    case 5:
                        fwcrc += (cc & 0xFF) << 16;
                        break;
                    case 6:
                        fwcrc += (cc & 0xFF) << 8;
                        break;
                    case 7:
                        fwcrc += (cc & 0xFF);
                        break;
                    case 8:
                        patchentrypoint = (cc & 0xFF) << 24;
                        break;
                    case 9:
                        patchentrypoint += (cc & 0xFF) << 16;
                        break;
                    case 10:
                        patchentrypoint += (cc & 0xFF) << 8;
                        break;
                    case 11:
                        patchentrypoint += (cc & 0xFF);
                        String sFwcrc = String.format("%08X", fwcrc);

                        System.out.println(String.format("Firmware version: %d.%d.%d.%d, CRC: 0x%s, entry point: 0x%08X", fwversion[0], fwversion[1], fwversion[2], fwversion[3], sFwcrc, patchentrypoint));
                        LOGGER.log(Level.INFO, String.format("Firmware version %d.%d.%d.%d | CRC %s\n", fwversion[0], fwversion[1], fwversion[2], fwversion[3], sFwcrc));
                        MainFrame.mainframe.setFirmwareID(sFwcrc);
                        GoIdleState();
                        break;
                }
                dataIndex++;
                break;

            default:
                GoIdleState();
                break;
        }
    }

    @Override
    public ksoloti_core getTargetProfile() {
        return targetProfile;
    }

}
