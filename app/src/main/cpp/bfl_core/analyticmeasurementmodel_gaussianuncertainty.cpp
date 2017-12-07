// $Id: analyticmeasurementmodel_gaussianuncertainty.cpp 29495 2008-08-13 12:57:49Z tdelaet $
// Copyright (C) 2002 Klaas Gadeyne <first dot last at gmail dot com>
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation; either version 2.1 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
//

#include "include/analyticmeasurementmodel_gaussianuncertainty.h"

namespace bfl_core
{
    // Constructor
    AnalyticMeasurementModelGaussianUncertainty::AnalyticMeasurementModelGaussianUncertainty
            (AnalyticConditionalGaussian *Measurementpdf)
            : MeasurementModel<Eigen::Vector3d, Eigen::Vector3d>(Measurementpdf)
    {}

    // Destructor
    AnalyticMeasurementModelGaussianUncertainty::~AnalyticMeasurementModelGaussianUncertainty()
    {}


    Eigen::Matrix3d
    AnalyticMeasurementModelGaussianUncertainty::df_dxGet(const Eigen::Vector3d &u,
                                                          const Eigen::Vector3d &x)
    {
        MeasurementPdfGet()->ConditionalArgumentSet(0, x);
        if (MeasurementPdfGet()->NumConditionalArgumentsGet() == 2)
            MeasurementPdfGet()->ConditionalArgumentSet(1, u);
        return dynamic_cast<AnalyticConditionalGaussian *>(MeasurementPdfGet())->dfGet(0);
    }


    Eigen::Vector3d
    AnalyticMeasurementModelGaussianUncertainty::PredictionGet(const Eigen::Vector3d &u,
                                                               const Eigen::Vector3d &x)
    {
        MeasurementPdfGet()->ConditionalArgumentSet(0, x);
        if (MeasurementPdfGet()->NumConditionalArgumentsGet() == 2)
            MeasurementPdfGet()->ConditionalArgumentSet(1, u);
        return MeasurementPdfGet()->ExpectedValueGet();
    }

    Eigen::Matrix3d
    AnalyticMeasurementModelGaussianUncertainty::CovarianceGet(const Eigen::Vector3d &u,
                                                               const Eigen::Vector3d &x)
    {
        MeasurementPdfGet()->ConditionalArgumentSet(0, x);
        if (MeasurementPdfGet()->NumConditionalArgumentsGet() == 2)
            MeasurementPdfGet()->ConditionalArgumentSet(1, u);
        return dynamic_cast<AnalyticConditionalGaussian *>(MeasurementPdfGet())->CovarianceGet();
    }

}
