clear all
clc
x = serialport('COM5',115200,"StopBits",1,"Parity","none","DataBits",8,"FlowControl","none")


%%
accel = zeros(3,61);
gyro = zeros(3,61);
magn = zeros(3,61);

tic
while true
    %a(n_comms+1) = typecast(uint8(fread(x,4)),'single');
    if(x.NumBytesAvailable >= 3)
        x.NumBytesAvailable;
        read = fgets(x);
        i = 1;
        while read(i)~='{'
            i = i+1;
        end
        read = read(i:end);
        try
        json_struct = jsondecode(read);
        catch
        json_struct.accel = accel(:,end);
        json_struct.gyro = gyro(:,end);
        json_struct.magn = magn(:,end);
        end
        
        accel = [accel,json_struct.accel];
        gyro = [gyro,json_struct.gyro];
        magn = [magn,json_struct.magn/norm(json_struct.magn)];
    end
    if toc > 1
        f1 = figure(1);
        clf(f1)
        title("Accelerometer");
        ylim([-1,1]);
        hold on
        plot(accel(1,end-60:end));
        plot(accel(2,end-60:end));
        plot(accel(3,end-60:end));
        legend("x","y","z");

        f2 = figure(2);
        clf(f2)
        title("Gyroscope");
        ylim([-1,1]);
        hold on
        plot(gyro(1,end-60:end));
        plot(gyro(2,end-60:end));
        plot(gyro(3,end-60:end));
        legend("x","y","z");

        f3 = figure(3);
        clf(f3)
        title("Magnetometer");
        ylim([-1,1]);
        hold on
        plot(magn(1,end-60:end));
        plot(magn(2,end-60:end));
        plot(magn(3,end-60:end));
        legend("x","y","z");
        tic
    end
    %message = convertCharsToStrings(native2unicode(a))
    %typecast(a,'single')
end

