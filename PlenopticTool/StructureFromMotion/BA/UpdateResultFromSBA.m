function StructureBag = UpdateResultFromSBA(p,ncams,cnp,npts,pnp,R0vector,StructureBag,QueryImgInfo)
    cams=reshape(p(1:ncams*cnp), cnp, ncams)';
    % derive the full (local) unit quaternions from their vector parts and combine with the initial rotation
    cams=[zeros(ncams, 1), cams]; % augment with a leading column
    for i=1:ncams
        % note that the cams column indices below are incremented by one to account for the extra column above!
        qrl=[sqrt(1 - (cams(i, 2)^2 + cams(i, 3)^2 + cams(i, 4)^2)), cams(i, 2), cams(i, 3), cams(i, 4)];
        qr0=R0vector((i-1)*4+1:i*4)';
        tmp=[qrl(1)*qr0(1) - dot(qrl(2:4), qr0(2:4)),  qrl(1)*qr0(2:4) + qr0(1)*qrl(2:4) + cross(qrl(2:4), qr0(2:4))];
        if(tmp(1)<0)
            tmp=-tmp;
        end  % ensure scalar part is non-negative
        cams(i, 1:4)=tmp;
    end

    RegisterFrame    = QueryImgInfo.registered';
    for n = 1:ncams
        Rh1 = quat2rot( cams(n, 1:4));
        th1 = cams(n, 5:end)';
        Rh  = Rh1';
        th = -Rh1'*th1;
        StructureBag.poses{RegisterFrame(n)} = [Rh th];  % update the StructureBag according to the descending order.
    end
    pts3DBA=reshape(p(ncams*cnp+1:ncams*cnp+npts*pnp), pnp, npts)';
    %StructureBag.Pts3DNew = pts3DBA;% Debug.
    StructureBag.Pts3D    = pts3DBA; % update the 3d points according to the previous order.
end