#ifndef __NVKM_CLK_NVAA_H__
#define __NVKM_CLK_NVAA_H__

#include <subdev/bus.h>
#include <subdev/bus/hwsq.h>
#include <subdev/clock.h>

struct nvaa_clock_priv {
	struct nouveau_clock base;
	enum nv_clk_src csrc, ssrc, vsrc;
	u32 cctrl, sctrl;
	u32 ccoef, scoef;
	u32 cpost, spost;
	u32 vdiv;
};

int  nvaa_clock_ctor(struct nouveau_object *, struct nouveau_object *,
		     struct nouveau_oclass *, void *, u32,
		     struct nouveau_object **);

#endif
